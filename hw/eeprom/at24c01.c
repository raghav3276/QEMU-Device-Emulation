/*
 * AT24C01 is an I2C based EEPROM (128x8 bits)
 */

#include "hw/i2c/i2c.h"

#define EEPROM_SIZE 128

typedef struct {
	I2CSlave i2c;
	uint8_t i2c_command_byte;

	uint8_t eeprom_addr;
	uint8_t data[EEPROM_SIZE];
} AT24C01State;

static const VMStateDescription at24c01_vmstate = {
	.name = "at24c01",
	.version_id = 0,
	.minimum_version_id = 0,
	.minimum_version_id_old = 0,
	.fields = (VMStateField[]) {
		VMSTATE_I2C_SLAVE(i2c, AT24C01State),
		VMSTATE_UINT8(i2c_command_byte, AT24C01State),
		VMSTATE_UINT8(eeprom_addr, AT24C01State),
		VMSTATE_UINT8_ARRAY(data, AT24C01State, EEPROM_SIZE),
		VMSTATE_END_OF_LIST()
	}
};

static int at24c01_recv(I2CSlave *i2c)
{
	AT24C01State *s = FROM_I2C_SLAVE(AT24C01State, i2c);

	return s->data[s->eeprom_addr];
}

static int at24c01_send(I2CSlave *i2c, uint8_t data)
{
	AT24C01State *s = FROM_I2C_SLAVE(AT24C01State, i2c);

	if (s->i2c_command_byte) {
#ifdef VERBOSE
		printf("Requested EEPROM address : 0x%x\n", data);
#endif
		s->i2c_command_byte = 0;
		s->eeprom_addr = data;
		return 0;
	}

	if (s->eeprom_addr >= EEPROM_SIZE)
		return 1;

	s->data[s->eeprom_addr] = data;
	return 0;
}

static void at24c01_event(I2CSlave *i2c, enum i2c_event event)
{
	AT24C01State *s = FROM_I2C_SLAVE(AT24C01State, i2c);

	switch (event) {
	case I2C_START_SEND:
		s->i2c_command_byte = 1;
		break;
	default:
		break;
	}
}

static int at24c01_init(I2CSlave *dev)
{
	return 0;
}

static void at24c01_reset(DeviceState *dev)
{
	AT24C01State *s = FROM_I2C_SLAVE(AT24C01State, I2C_SLAVE(dev));

	s->eeprom_addr = 0;
	s->i2c_command_byte = 0;
}

static void at24c01_class_init(ObjectClass *klass, void *data)
{
	DeviceClass *dc		= DEVICE_CLASS(klass);
	I2CSlaveClass *k	= I2C_SLAVE_CLASS(klass);

	k->init = at24c01_init;
	k->event = at24c01_event;
	k->recv = at24c01_recv;
	k->send = at24c01_send;

	dc->reset = at24c01_reset;
	dc->vmsd = &at24c01_vmstate;
}

static const TypeInfo at24c01_info = {
	.name		= "at24c01",
	.parent		= TYPE_I2C_SLAVE,
	.instance_size	= sizeof(AT24C01State),
	.class_init	= at24c01_class_init
};

static void at24c01_register(void)
{
	type_register_static(&at24c01_info);
}

type_init(at24c01_register)
