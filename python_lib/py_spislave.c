#include <Python.h>
#include "structmember.h"
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/types.h>
#include <linux/ioctl.h>
#include <sys/ioctl.h>
#include "../driver/spi-slave-dev.h"

#define MAX_PATH	4096

// Macros needed for Python 3
#ifndef PyInt_Check
#define PyInt_Check	PyLong_Check
#define PyInt_FromLong	PyLong_FromLong
#define PyInt_AsLong	PyLong_AsLong
#define PyInt_Type	PyLong_Type
#endif

typedef struct {
	PyObject_HEAD;

	int		fd; /*file descriptor  /dev/spislaveX*/
	uint8_t		mode;
	uint32_t	tx_actual_length;
	uint32_t	rx_actual_length;
	uint32_t	max_speed;
	uint8_t		bits_per_word;
} SPIslave;

static PyObject*
SPIslave_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
	SPIslave *self;
	if ((self = (SPIslave *)type->tp_alloc(type, 0)) == NULL)
		return NULL;

	self->fd = -1;
	self->mode = 0;
	self->tx_actual_length = 0;
	self->rx_actual_length = 0;
	self->max_speed = 0;
	self->bits_per_word = 0;

	return (PyObject *)self;
}

static PyObject*
SPIslave_open(SPIslave* self, PyObject* args)
{
	int device;
	char path[MAX_PATH];
	uint32_t tmp32;
	uint8_t tmp8;


	if (self == NULL)
		printf("self is NULL\n");

	if (!PyArg_ParseTuple(args, "i", &device))
		return NULL;

	if (snprintf(path, MAX_PATH, "/dev/spislave%d", device) >= MAX_PATH) {
		PyErr_SetString(PyExc_OverflowError, "Device number is invalid.");
		return NULL;
	}

	self->fd = open(path, O_RDWR, 0);
	if (self->fd == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	if (ioctl(self->fd, SPISLAVE_RD_MODE, &tmp8) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->mode = tmp8;

	if (ioctl(self->fd, SPISLAVE_RD_MAX_SPEED, &tmp32) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->max_speed = tmp32;

	if (ioctl(self->fd, SPISLAVE_RD_BITS_PER_WORD, &tmp8) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->bits_per_word = tmp8;

	if (ioctl(self->fd, SPISLAVE_RD_TX_ACTUAL_LENGTH, &tmp32) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->tx_actual_length = tmp32;

	if (ioctl(self->fd, SPISLAVE_RD_RX_ACTUAL_LENGTH, &tmp32) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->rx_actual_length = tmp32;

	Py_RETURN_NONE;
}

static PyObject*
SPIslave_close(SPIslave* self)
{
	if ((self->fd != -1) && (close(self->fd) == -1)) {
		PyErr_SetFromErrno(PyExc_IOError);
	}

	self->fd = -1;
	self->mode = 0;
	self->tx_actual_length = 0;
	self->rx_actual_length = 0;
	self->max_speed = 0;
	self->bits_per_word = 0;

	Py_RETURN_NONE;
}

static void
SPIslave_dealloc(SPIslave *self)
{
	PyObject *ref = SPIslave_close(self);
	printf("spi slave dealoc python mod");

	Py_XDECREF(ref);;
}

static PyObject *
SPIslave_read(SPIslave* self, PyObject* args)
{

	return 0;
}

static PyObject *
SPIslave_write(SPIslave* self, PyObject* args)
{

	return Py_None;
}

static PyMethodDef SPIslave_methods[] =
{
	{"open", (PyCFunction)SPIslave_open, METH_VARARGS | METH_KEYWORDS, "open desc"},
	{"close", (PyCFunction)SPIslave_close, METH_NOARGS, "close desc"},
	{"read", (PyCFunction)SPIslave_read, METH_VARARGS, "read desc"},
	{"write", (PyCFunction)SPIslave_write, METH_VARARGS, "write desc"},
	{NULL},
};

static int
__SPIslave_set_mode(SPIslave *self)
{
	if (ioctl(self->fd, SPISLAVE_WR_MODE, &self->mode) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return -1;
	}
	return 0;
}

static __u8
__SPIslave_get_mode(SPIslave *self)
{
	__u8 mode;

	if (ioctl(self->fd, SPISLAVE_RD_MODE, &mode) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return -1;
	}

	if (mode < 0) {
		return -1;
	}

	return mode;
}

static PyObject*
SPIslave_get_mode(SPIslave *self, void *closure)
{
	PyObject *result = NULL;/*now is null dont use it*/

	__SPIslave_get_mode(self);
	return result;
}

static int
SPIslave_set_mode(SPIslave *self, PyObject *val, void *closure)
{
	__SPIslave_set_mode(self);

	return 0;
}

static PyObject*
SPIslave_get_bits_per_word(SPIslave *self, void *closure)
{
	PyObject *result = NULL; /**now is null dont use it*/

	return result;
}

static int
SPIslave_set_bits_per_word(SPIslave *self, PyObject *val, void *closure)
{

	return 0;
}

static PyObject*
SPIslave_get_max_speed(SPIslave *self, void *closure)
{
	PyObject *result = NULL; /*now is NULL dont use it*/

	return result;
}

static int
SPIslave_set_max_speed(SPIslave *self, PyObject *val, void *closure)
{

	return 0;
}

static PyObject*
SPIslave_get_tx_actual_length(SPIslave *self, void *closure)
{
	PyObject *result = NULL;


	return result;
}

static PyObject*
SPIslave_get_rx_actual_length(SPIslave *self, void *closure)
{
	PyObject *result = NULL;


	return result;
}

static PyObject*
SPIslave_get_SLAVE(SPIslave *self, void *closure)
{
	PyObject *result = NULL; /*now is NULL dont use it*/

	return result;
}

static int
SPIslave_set_SLAVE(SPIslave *self, PyObject *val, void *closure)
{

	return 0;
}

static PyObject*
SPIslave_get_CPOL(SPIslave *self, void *closure)
{
	PyObject *result = NULL; /*now is NULL dont use it*/

	return result;
}

static int
SPIslave_set_CPOL(SPIslave *self, PyObject *val, void *closure)
{

	return 0;
}

static PyObject*
SPIslave_get_CPHA(SPIslave *self, void *closure)
{
	PyObject *result = NULL; /*now is NULL dont use it*/

	return result;
}

static int
SPIslave_set_CPHA(SPIslave *self, PyObject *val, void *closure)
{

	return 0;
}

static PyObject*
SPIslave_get_NO_CS(SPIslave *self, void *closure)
{
	PyObject *result = NULL; /*now is NULL dont use it*/

	return result;
}

static int
SPIslave_set_NO_CS(SPIslave *self, PyObject *val, void *closure)
{

	return 0;
}

static PyObject*
SPIslave_get_CS_HIGH(SPIslave *self, void *closure)
{
	PyObject *result = NULL; /*now is NULL dont use it*/

	return result;
}

static int
SPIslave_set_CS_HIGH(SPIslave *self, PyObject *val, void *closure)
{

	return 0;
}

static PyObject*
SPIslave_get_LSB_FIRST(SPIslave *self, void *closure)
{
	PyObject *result = NULL; /*now is NULL dont use it*/

	return result;
}

static int
SPIslave_set_LSB_FIRST(SPIslave *self, PyObject *val, void *closure)
{

	return 0;
}

static PyGetSetDef SPIslave_getset[] = {
	{"mode",
	(getter)SPIslave_get_mode,
	(setter)SPIslave_set_mode,
	"spislave mode"},

	{"SPIslave_SLAVE",
	(getter)SPIslave_get_SLAVE,
	(setter)SPIslave_set_SLAVE,
	""},

	{"SPIslave_CPHA",
	(getter)SPIslave_get_CPHA,
	(setter)SPIslave_set_CPHA,
	""},

	{"SPIslave_CPOL",
	(getter)SPIslave_get_CPOL,
	(setter)SPIslave_set_CPOL,
	""},

	{"SPIslave_NO_CS",
	(getter)SPIslave_get_NO_CS,
	(setter)SPIslave_set_NO_CS,
	""},

	{"SPIslave_CS_HIGH",
	(getter)SPIslave_get_CS_HIGH,
	(setter)SPIslave_set_CS_HIGH,
	""},

	{"SPIslave_LSB_FIRST",
	(getter)SPIslave_get_LSB_FIRST,
	(setter)SPIslave_set_LSB_FIRST,
	""},

	{"max_speed",
	(getter)SPIslave_get_max_speed,
	(setter)SPIslave_set_max_speed,
	"spislave max speed"},

	{"bits_per_word",
	(getter)SPIslave_get_bits_per_word,
	(setter)SPIslave_set_bits_per_word,
	"spislave_bits_per_word"},

	{"tx_actual_length",
	(getter)SPIslave_get_tx_actual_length,
	(setter)NULL, "spislave_tx_actual_length"},

	{"rx_actual_length",
	(getter)SPIslave_get_rx_actual_length,
	(setter)NULL, "spislave_rx_actual_length"},

	{NULL},
};

static int
SPIslave_init(SPIslave *self, PyObject *args)
{
	int device = -1;
	char path[MAX_PATH];

	if (!PyArg_ParseTuple(args, "i", &device))
		return -1;

	if (snprintf(path, MAX_PATH, "/dev/spislave%d", device) >= MAX_PATH) {
		PyErr_SetString(PyExc_OverflowError, "Device number is invalid.");
		return -1;
	}

	if (device >= 0) {
		SPIslave_open(self, args);
		if (PyErr_Occurred())
			return -1;
	}

	return 0;
}

static PyTypeObject SPIslave_type = {
	PyVarObject_HEAD_INIT(NULL, 0)
	"SPIslave",			/* tp_name */
	sizeof(SPIslave),		/* tp_basicsize */
	0,				/* tp_itemsize */
	(destructor)SPIslave_dealloc,	/* tp_dealloc */
	0,				/* tp_print */
	0,				/* tp_getattr */
	0,				/* tp_setattr */
	0,				/* tp_compare */
	0,				/* tp_repr */
	0,				/* tp_as_number */
	0,				/* tp_as_sequence */
	0,				/* tp_as_mapping */
	0,				/* tp_hash */
	0,				/* tp_call */
	0,				/* tp_str */
	0,				/* tp_getattro */
	0,				/* tp_setattro */
	0,				/* tp_as_buffer */
	Py_TPFLAGS_DEFAULT,		/* tp_flags */
	0,				/* tp_doc */
	0,				/* tp_traverse */
	0,				/* tp_clear */
	0,				/* tp_richcompare */
	0,				/* tp_weaklistoffset */
	0,				/* tp_iter */
	0,				/* tp_iternext */
	SPIslave_methods,		/* tp_methods */
	0,				/* tp_members */
	SPIslave_getset,		/* tp_getset */
	0,				/* tp_base */
	0,				/* tp_dict */
	0,				/* tp_descr_get */
	0,				/* tp_descr_set */
	0,				/* tp_dictoffset */
	(initproc)SPIslave_init,	/* tp_init */
	0,				/* tp_alloc */
	SPIslave_new,			/* tp_new */
};


static struct PyModuleDef SPIslave_module = {
	PyModuleDef_HEAD_INIT,
	"SPIslave",
	"Module for SPI slave",
	-1,
	NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC
PyInit_SPIslave(void)
{
	PyObject *m;

	SPIslave_type.tp_new = PyType_GenericNew;
	if (PyType_Ready(&SPIslave_type) < 0)
		return NULL;

	m = PyModule_Create(&SPIslave_module);
	if (m == NULL)
		return NULL;

	Py_INCREF(&SPIslave_type);
	PyModule_AddObject(m, "SPIslave", (PyObject *)&SPIslave_type);

	return m;
}
