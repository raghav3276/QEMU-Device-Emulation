/*
 * MMA7660fc I2C device 
 *
 * Copyright (c) 2014 c-dac.
 * Written by Dileep K Panjala & A Raghavendra Rao
 *
 * This code is licensed under the GNU GPL v2.
 *
 * Contributions after 2014-07-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include "hw/i2c/i2c.h"
#include <X11/Xlib.h>
#include <pthread.h>

#define NREGS 11

/* abs of new coordinates minus new coordinates */
#define SHAKE_THRESHOLD 100

#define PTR_X_MAX 1024
#define PTR_Y_MAX 768

#define MMA7660_MEASUREMENT_DELAY 100

#define ALERT 6

#define MMA7660_STANDBY_MODE	1
#define MMA7660_ACTIVE_MODE 	2
#define MMA7660_TEST_MODE		3


enum mma_regs {
	XOUT = 0x00,
	YOUT = 0x01,
	ZOUT = 0x02,
	TILT = 0x03,
	SRST = 0x04,
	SPCNT = 0x05,
	INTSU = 0x06,
	MODE = 0x07,
	SR = 0x08,
	PDET = 0x09,
	PD = 0x0A
};

typedef struct {
	I2CSlave i2c;
	bool addr_byte;

	uint8_t reg;
	uint8_t mma7660_regs[NREGS];

	uint8_t mode;
	uint8_t samples_ps;

	pthread_t tilt_update_thread_tid;
	pthread_t xyz_update_thread_tid;

} MMA7660state;

struct VMStateDescription vmstate_mma7660 = {
	.name = "mma7660",
	.version_id = 2,
	.minimum_version_id = 1,
	.minimum_version_id_old = 1,
	.fields = (VMStateField[]) {
		VMSTATE_I2C_SLAVE(i2c, MMA7660state),
		VMSTATE_BOOL(addr_byte, MMA7660state),
		VMSTATE_UINT8(reg, MMA7660state),
		VMSTATE_UINT8_ARRAY(mma7660_regs, MMA7660state,11),
		VMSTATE_UINT8(samples_ps, MMA7660state),
		VMSTATE_END_OF_LIST()
	}	
};

static void mma7660_clear_xyz(MMA7660state *s)
{
	s->mma7660_regs[XOUT] = 0x00;
	s->mma7660_regs[YOUT] = 0x00;
	s->mma7660_regs[ZOUT] = 0x00;
}

static void mma7660_query_pointer(int *x, int *y, unsigned int *mask)
{
	Display *dpy;
	Window root, child;
	int childX, childY;

	dpy = XOpenDisplay(NULL);
	XQueryPointer(dpy, DefaultRootWindow(dpy),
					&root, &child,
					x, y, &childX,
					&childY, mask);
	XCloseDisplay(dpy);
}

/* Thread which continuously updates X,Y and Z axes */
static void *xyz_update_thread(void *mma7660_state)
{
	int x, y;
	unsigned int mask;
	MMA7660state *s = mma7660_state;

	while (1) {

		usleep(1000 * 1000 / s->samples_ps);

		if(s->mode == MMA7660_ACTIVE_MODE) {

			mma7660_query_pointer(&x, &y, &mask);

			/* Update X */
			s->mma7660_regs[XOUT] |= (1 << ALERT);	/* Set the alert bit */
			s->mma7660_regs[XOUT] = x;				/* Update the register */
			s->mma7660_regs[XOUT] &= 0x3f;			/* Reset the alert bit,
			 	 	 	 	 	 	 	 	 	 	 with granularity of 6-bits/value */

			/* Update Y */
			s->mma7660_regs[YOUT] |= (1 << ALERT);
			s->mma7660_regs[YOUT] = y;
			s->mma7660_regs[YOUT] &= 0x3f;

			/* Update Z */
			s->mma7660_regs[ZOUT] |= (1 << ALERT);
			s->mma7660_regs[ZOUT] = x ^ y;
			s->mma7660_regs[ZOUT] &= 0x3f;
		}
	}

	pthread_exit(NULL);
}

static void *tilt_update_thread(void *mma7660_state)
{
	int x, y;
	int x_prev = 0, y_prev = 0;
	unsigned int mask;
	MMA7660state *s = mma7660_state;

	while (1) {

		/* Debounce delay */
		usleep((1000 * 1000 / s->samples_ps) * (s->mma7660_regs[SR] >> 5));

		if (s->mode == MMA7660_ACTIVE_MODE) {

			mma7660_query_pointer(&x, &y, &mask);

			s->mma7660_regs[TILT] |= ALERT;

			/* Shake */
			if (abs(x - x_prev) > SHAKE_THRESHOLD
					|| abs(y - y_prev) > SHAKE_THRESHOLD) {

				s->mma7660_regs[TILT] |= (1 << 7);

				x_prev = x;
				y_prev = y;
			} else {
				s->mma7660_regs[TILT] &= 0x7f;
			}

			/* Tap : Mouse middle button */
			if (mask & Button2Mask && s->samples_ps == 120)
				s->mma7660_regs[TILT] |= (1 << 5);
			else
				s->mma7660_regs[TILT] &= 0xDF;

			/* Up */
			if ((y < (PTR_Y_MAX / 2)) && (x < (PTR_X_MAX / 2)))
				s->mma7660_regs[TILT] = (s->mma7660_regs[TILT] & 0xE3) | 0x18;

			/* Down */
			else if ((y > (PTR_Y_MAX / 2)) && (x < (PTR_X_MAX / 2)))
				s->mma7660_regs[TILT] = (s->mma7660_regs[TILT] & 0xE3) | 0x14;

			/* Right */
			else if ((y < (PTR_Y_MAX / 2)) && (x > (PTR_X_MAX / 2)))
				s->mma7660_regs[TILT] = (s->mma7660_regs[TILT] & 0xE3) | 0x08;

			/* Left */
			else if ((y > (PTR_Y_MAX / 2)) && (x > (PTR_X_MAX / 2)))
				s->mma7660_regs[TILT] = (s->mma7660_regs[TILT] & 0xE3) | 0x04;

			/* Front */
			if (y > (PTR_Y_MAX / 2))
				s->mma7660_regs[TILT] = (s->mma7660_regs[TILT] & 0xFC) | 0x01;

			/* Back */
			else
				s->mma7660_regs[TILT] = (s->mma7660_regs[TILT] & 0xFC) | 0x10;

			s->mma7660_regs[TILT] &= 0xBF;
		}
	}

	pthread_exit(NULL);
}

static int mma7660_dev_recv(I2CSlave *i2c)
{
	MMA7660state *s = FROM_I2C_SLAVE(MMA7660state,i2c);

	return s->mma7660_regs[s->reg];
}

static void mma7660_manage_mode_reg(MMA7660state *s, uint8_t data)
{
	mma7660_clear_xyz(s);
	s->mma7660_regs[TILT] = 0x00;

	/* Figure out the mode */
	switch (data & 0x07) {
	case 0x00:
		s->mode = MMA7660_STANDBY_MODE;
		break;
	case 0x04:
		/* Previous state must be standby to enter test mode */
		if (s->mode != MMA7660_ACTIVE_MODE) {
			s->mode = MMA7660_TEST_MODE;
		}
		break;
	case 0x01:
		/* Previous state should not be a test mode */
		if (s->mode != MMA7660_TEST_MODE) {
			s->mode = MMA7660_ACTIVE_MODE;
		}
	}
}

static void mma7660_manage_sr_reg(MMA7660state *s, uint8_t data)
{
	switch (data & 0x07) {
	case 0: s->samples_ps = 120; break;
	case 1: s->samples_ps = 64; break;
	case 2: s->samples_ps = 32; break;
	case 3: s->samples_ps = 16; break;
	case 4: s->samples_ps = 8; break;
	case 5: s->samples_ps = 4; break;
	case 6: s->samples_ps = 2; break;
	case 7: s->samples_ps = 1; break;
	}
}

static void mma7660_write(MMA7660state *s, uint8_t data)
{
	switch (s->reg) {

	case XOUT:
	case YOUT:
	case ZOUT:
		if (s->mode != MMA7660_TEST_MODE)
			return;
		break;

	case TILT:	/* Read-only register */
		return;

	case SRST:	/* Read-only register */
		return;

	case MODE:
		mma7660_manage_mode_reg(s, data);
		break;

	case SR:
		if (s->mode != MMA7660_STANDBY_MODE)
			return;
		mma7660_manage_sr_reg(s, data);

	default:
		/* Only standby mode allows other register to be written */
		if (s->mode != MMA7660_STANDBY_MODE)
			return;
	}

	/* Write the data to the register */
	s->mma7660_regs[s->reg] = data;
}

static int mma7660_dev_send(I2CSlave *i2c, uint8_t data)
{
	MMA7660state *s = FROM_I2C_SLAVE(MMA7660state,i2c);

	if(s->addr_byte) {
		s->addr_byte = 0;
		s->reg = data;
		return 0;
	}

	if (s->reg >= NREGS)
		return 1;

	mma7660_write(s, data);

	return 0;
}

static void mma7660_dev_event(I2CSlave *i2c , enum i2c_event event)
{
	MMA7660state *s = FROM_I2C_SLAVE(MMA7660state, i2c);

	switch(event) {
	case I2C_START_SEND:
		s->addr_byte = true;
		break;

	default:
		break;
	}
}

static void mma7660_dev_reset(DeviceState *state)
{
	int i;
	MMA7660state *s = FROM_I2C_SLAVE(MMA7660state, I2C_SLAVE(state));
	
	s->addr_byte = false;
	s->samples_ps = 120;
	s->mode = MMA7660_STANDBY_MODE;

	for (i = 0; i < NREGS; i++)
		s->mma7660_regs[i] = 0x00;

	s->mma7660_regs[SRST] = 0x01;
}

static int mma7660_dev_init(I2CSlave *i2c)
{
	int retval;
	MMA7660state *s = FROM_I2C_SLAVE(MMA7660state , i2c);

	s->samples_ps = 120;
	s->mode = MMA7660_STANDBY_MODE;

	/* Start updating X,Y,Z values */
	retval = pthread_create(&s->xyz_update_thread_tid, NULL, xyz_update_thread, s);
	if (retval)
		goto xyz_thread_fail;

	/* Start updating the TILT register based on X,Y,Z values */
	retval = pthread_create(&s->tilt_update_thread_tid, NULL, tilt_update_thread, s);
	if (retval)
		goto tilt_thread_fail;

	return 0;

tilt_thread_fail:
	pthread_cancel(s->xyz_update_thread_tid);
xyz_thread_fail:
	return retval;
}

static void mma7660_class_init(ObjectClass *klass , void *data)
{
	DeviceClass *dc = DEVICE_CLASS(klass);
	I2CSlaveClass *slave = I2C_SLAVE_CLASS(klass);

	slave->init = mma7660_dev_init;
	slave->send = mma7660_dev_send;
	slave->recv = mma7660_dev_recv;
	slave->event = mma7660_dev_event;
	dc->reset = mma7660_dev_reset;
	dc->vmsd = &vmstate_mma7660;
}

static const TypeInfo mma7660_info ={
	.name = "mma7660",
	.parent = TYPE_I2C_SLAVE,
	.instance_size = sizeof(MMA7660state),
	.class_init = mma7660_class_init,
};

static void mma7660_register_types(void)
{
	type_register_static(&mma7660_info);
}

type_init(mma7660_register_types)
