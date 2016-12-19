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

/* Macros needed for Python 3*/
#ifndef PyInt_Check
#define PyInt_Check	PyLong_Check
#define PyInt_FromLong	PyLong_FromLong
#define PyInt_AsLong	PyLong_AsLong
#define PyInt_Type	PyLong_Type
#endif

/*
 * TODO:
 * add support for full-duplex transfer,
 */

PyDoc_STRVAR(SPIslave_doc, "This module defines an object type that allows\n"
"SPIslave transactions on hosts running the Linux kernel. The host kernel\n"
"must have SPIslave core and SPIslave device interface.\n"
"Author: Patryk Mezydlo"
"E-mail: mezydlo.p@gmail.com");

PyDoc_STRVAR(SPIslave_open_doc,"open(device)\n\n"
			       "Connects the object to the specified\n"
			       "SPIslave device.\n"
			       "open(X) will open /dev/spislave<X>");

PyDoc_STRVAR(SPIslave_close_doc,"close()\n\n"
				"Disconnects the object from the interface\n");

PyDoc_STRVAR(SPIslave_read_doc,"read(len) -> [values]\n\n"
			       "Read len bytes from SPIslave device.\n");

PyDoc_STRVAR(SPIslave_write_doc,"write([values])\n\n"
				"Write bytes to SPIslave device.\n");

typedef struct {
	PyObject_HEAD;

	int		fd; /*file descriptor  /dev/spislaveX*/
	uint8_t		mode;
	uint32_t	tx_actual_length;
	uint32_t	rx_actual_length;
	uint32_t	max_speed;
	uint8_t		bits_per_word;
} SPIslave;

static PyObject *SPIslave_new(PyTypeObject *type, PyObject *args,
			      PyObject *kwds)
{
	SPIslave *self;

	self = (SPIslave *)type->tp_alloc(type, 0);
	if (self == NULL)
		return NULL;

	self->fd = -1;
	self->mode = 0;
	self->tx_actual_length = 0;
	self->rx_actual_length = 0;
	self->max_speed = 0;
	self->bits_per_word = 0;

	return (PyObject *)self;
}

static PyObject *SPIslave_open(SPIslave *self, PyObject *args)
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
		PyErr_SetString(PyExc_OverflowError,
				"Device number is invalid.");
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

static PyObject *SPIslave_close(SPIslave *self)
{
	if ((self->fd != -1) && (close(self->fd) == -1)) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	self->fd = -1;
	self->mode = 0;
	self->tx_actual_length = 0;
	self->rx_actual_length = 0;
	self->max_speed = 0;
	self->bits_per_word = 0;

	Py_RETURN_NONE;
}

static void SPIslave_dealloc(SPIslave *self)
{
	PyObject *ref = SPIslave_close(self);

	printf("spi slave dealoc python mod");
	Py_XDECREF(ref);
}

static PyObject *SPIslave_read(SPIslave *self, PyObject *args)
{
	uint8_t rx[MAX_PATH];
	int status, len, i;
	PyObject *list;

	if (!PyArg_ParseTuple(args, "i:read", &len))
		return NULL;

	if (len < 1)
		len = 1;
	else if ((unsigned)len > sizeof(rx))
		len = sizeof(rx);

	memset(rx, 0, sizeof(rx));
	status = read(self->fd, &rx[0], len);

	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	if (status != len) {
		perror("short read");
		return NULL;
	}

	list = PyList_New(len);

	for (i = 0; i < len; i++) {
		PyObject *val = Py_BuildValue("l", (long)rx[i]);

		PyList_SET_ITEM(list, i, val);
	}

	return list;
}

static char *wrmsg_list0 = "Empty argument list.";
static char *wrmsg_listmax = "Argument list size exceeds %d bytes.";
static char *wrmsg_val = "Non-Int/Long value in argument: %x.";

static PyObject *SPIslave_write(SPIslave *self, PyObject *args)
{
	int status;
	uint16_t i, len;
	uint8_t tx[MAX_PATH];
	PyObject *obj, *seq;
	char wrmsg_text[MAX_PATH];

	if (!PyArg_ParseTuple(args, "O:write", &obj))
		return NULL;

	seq = PySequence_Fast(obj, "expected a sequence");
	len = PySequence_Fast_GET_SIZE(obj);
	if (!seq || len <= 0) {
		PyErr_SetString(PyExc_OverflowError, wrmsg_list0);
		return NULL;
	}

	if (len > MAX_PATH) {
		snprintf(wrmsg_text, sizeof(wrmsg_text) - 1, wrmsg_listmax,
			 MAX_PATH);
		PyErr_SetString(PyExc_OverflowError, wrmsg_text);
		return NULL;
	}

	for (i = 0; i < len; i++) {
		PyObject *val = PySequence_Fast_GET_ITEM(seq, i);

		if (PyInt_Check(val))
			tx[i] = (__u8)PyInt_AsLong(val);
		else {
			snprintf(wrmsg_text, sizeof(wrmsg_text) - 1, wrmsg_val,
				 val);
			PyErr_SetString(PyExc_TypeError, wrmsg_text);
			return NULL;
		}
	}

	Py_DECREF(seq);
	status = write(self->fd, &tx[0], len);

	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	if (status != len) {
		perror("short write");
		return NULL;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

static PyMethodDef SPIslave_methods[] = {
	{"open",
	 (PyCFunction)SPIslave_open,
	 METH_VARARGS | METH_KEYWORDS,
	 SPIslave_open_doc},

	{"close",
	 (PyCFunction)SPIslave_close,
	 METH_NOARGS,
	 SPIslave_close_doc},

	{"read",
	 (PyCFunction)SPIslave_read,
	 METH_VARARGS,
	 SPIslave_read_doc},

	{"write",
	 (PyCFunction)SPIslave_write,
	 METH_VARARGS,
	 SPIslave_write_doc},

	{NULL},
};

static int __SPIslave_set_mode(SPIslave *self, __u8 tmp)
{
	if (ioctl(self->fd, SPISLAVE_WR_MODE, &tmp) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return -1;
	}
	return 0;
}

static PyObject *SPIslave_get_mode(SPIslave *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", self->mode);

	Py_INCREF(result);
	return result;
}

static int SPIslave_set_mode(SPIslave *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError, "value is invalid");
		return -1;
	}

	if (PyLong_Check(val)) {
		tmp = PyInt_AsLong(val);
	} else {
		PyErr_SetString(PyExc_TypeError, "value must be integer");
		return -1;
	}

	if (tmp >= 0x3F) {
		PyErr_SetString(PyExc_TypeError,
				"the mode must be between 0 and 63.");
		return -1;
	}

	if (self->mode != tmp) {
		if (__SPIslave_set_mode(self, tmp) < 0)
			return -1;

		self->mode = tmp;
	}

	return 0;
}

static PyObject *SPIslave_get_bits_per_word(SPIslave *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", self->bits_per_word);

	Py_INCREF(result);
	return result;
}

static int SPIslave_set_bits_per_word(SPIslave *self, PyObject *val,
				      void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError, "value is invalid");
		return -1;
	}

	if (PyLong_Check(val)) {
		tmp = PyInt_AsLong(val);
	} else {
		PyErr_SetString(PyExc_TypeError, "value must be integer");
		return -1;
	}

	if (tmp > 64) {
		PyErr_SetString(PyExc_TypeError,
				"the bits per word must be between 0 and 64.");
		return -1;
	}

	if (self->bits_per_word != tmp) {
		if (ioctl(self->fd, SPISLAVE_WR_BITS_PER_WORD, &tmp) == -1) {
			PyErr_SetFromErrno(PyExc_IOError);
			return -1;
		}
		self->bits_per_word = tmp;
	}

	return 0;
}

static PyObject *SPIslave_get_max_speed(SPIslave *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", self->max_speed);

	Py_INCREF(result);
	return result;
}

static int SPIslave_set_max_speed(SPIslave *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError, "value is invalid");
		return -1;
	}

	if (PyLong_Check(val)) {
		tmp = PyInt_AsLong(val);
	} else {
		PyErr_SetString(PyExc_TypeError, "value must be integer");
		return -1;
	}

	if (self->max_speed != tmp) {
		if (ioctl(self->fd, SPISLAVE_WR_MAX_SPEED, &tmp) == -1) {
			PyErr_SetFromErrno(PyExc_IOError);
			return -1;
		}
		self->max_speed = tmp;
	}

	return 0;
}

static PyObject *SPIslave_get_tx_actual_length(SPIslave *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", self->tx_actual_length);

	Py_INCREF(result);
	return result;
}

static PyObject *SPIslave_get_rx_actual_length(SPIslave *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", self->rx_actual_length);

	Py_INCREF(result);
	return result;
}

static PyObject *SPIslave_get_SLAVE(SPIslave *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPISLAVE_SLAVE)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static int SPIslave_set_SLAVE(SPIslave *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError, "value is invalid");
		return -1;
	} else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError, "the slave must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPISLAVE_SLAVE;
	else
		tmp = self->mode & ~SPISLAVE_SLAVE;

	if (__SPIslave_set_mode(self, tmp) < 0)
		return -1;

	self->mode = tmp;

	return 0;
}

static PyObject *SPIslave_get_CPOL(SPIslave *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPISLAVE_CPOL)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static int SPIslave_set_CPOL(SPIslave *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError, "value is invalid");
		return -1;
	} else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError, "the CPOL must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPISLAVE_CPOL;
	else
		tmp = self->mode & ~SPISLAVE_CPOL;

	if (__SPIslave_set_mode(self, tmp) < 0)
		return -1;

	self->mode = tmp;

	return 0;
}

static PyObject *SPIslave_get_CPHA(SPIslave *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPISLAVE_CPHA)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static int SPIslave_set_CPHA(SPIslave *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError, "value is invalid");
		return -1;
	} else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError, "the CPHA must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPISLAVE_CPHA;
	else
		tmp = self->mode & ~SPISLAVE_CPHA;

	if (__SPIslave_set_mode(self, tmp) < 0)
		return -1;

	self->mode = tmp;

	return 0;
}

static PyObject *SPIslave_get_NO_CS(SPIslave *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPISLAVE_NO_CS)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static int SPIslave_set_NO_CS(SPIslave *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError, "value is invalid");
		return -1;
	} else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError, "the NO_CS must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPISLAVE_NO_CS;
	else
		tmp = self->mode & ~SPISLAVE_NO_CS;

	if (__SPIslave_set_mode(self, tmp) < 0)
		return -1;

	self->mode = tmp;

	return 0;
}

static PyObject *SPIslave_get_CS_HIGH(SPIslave *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPISLAVE_CS_HIGH)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static int SPIslave_set_CS_HIGH(SPIslave *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError, "value is invalid");
		return -1;
	} else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError, "the CS_HIGH must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPISLAVE_CS_HIGH;
	else
		tmp = self->mode & ~SPISLAVE_CS_HIGH;

	if (__SPIslave_set_mode(self, tmp) < 0)
		return -1;

	self->mode = tmp;

	return 0;
}

static PyObject *SPIslave_get_LSB_FIRST(SPIslave *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPISLAVE_LSB_FIRST)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static int SPIslave_set_LSB_FIRST(SPIslave *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError, "value is invalid");
		return -1;
	} else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
				"the LSB_FIRST must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPISLAVE_LSB_FIRST;
	else
		tmp = self->mode & ~SPISLAVE_LSB_FIRST;

	if (__SPIslave_set_mode(self, tmp) < 0)
		return -1;

	self->mode = tmp;

	return 0;
}

static PyGetSetDef SPIslave_getset[] = {
	{"mode",
	(getter)SPIslave_get_mode,
	(setter)SPIslave_set_mode,
	"Defines basic setting of SPI transfer.\n"},

	{"SLAVE",
	(getter)SPIslave_get_SLAVE,
	(setter)SPIslave_set_SLAVE,
	"Spi slave subsystem allows to work on both master or slave mode.\n"
	"You need to state which mode you want to use before transfer.\n"
	"Master mode is the main mode and is set by default.\n"},

	{"CPHA",
	(getter)SPIslave_get_CPHA,
	(setter)SPIslave_set_CPHA,
	"That is CPHA=0 means sampling on the first clock edge, while CPHA=1\n"
	"means sampling on the second clock edge, regardless of whether that\n"
	"clock edge is rising or falling.\n"},

	{"CPOL",
	(getter)SPIslave_get_CPOL,
	(setter)SPIslave_set_CPOL,
	"If CPOL is zero, than SCLK is normally low, and the first clock edge\n"
	"is a rising edge. If CPOL is one, SCLK is normally high, and the\n"
	"first clock edge is a falling edge.\n"},

	{"NO_CS",
	(getter)SPIslave_get_NO_CS,
	(setter)SPIslave_set_NO_CS,
	"If NO_CS is zero, that CS is active, when NO_CS is set,\n"
	"CS line is not active.\n"},

	{"CS_HIGH",
	(getter)SPIslave_get_CS_HIGH,
	(setter)SPIslave_set_CS_HIGH,
	"That is CS_HIGH means when the CS line state is high actives the\n"
	"receiving device.\n"},

	{"LSB_FIRST",
	(getter)SPIslave_get_LSB_FIRST,
	(setter)SPIslave_set_LSB_FIRST,
	"The bit justification used to transfer spi words. Zero indicates\n"
	"MSB-first; other values indicate the less common LSB-first\n"
	"encoding. MSB-first is set by default.\n"},

	{"max_speed",
	(getter)SPIslave_get_max_speed,
	(setter)SPIslave_set_max_speed,
	"Only in master mode. The maximum SPI transfer speed in Hz. The spi\n"
	"controller does not have to set this setting. Not every controller\n"
	"supports it.\n"},

	{"bits_per_word",
	(getter)SPIslave_get_bits_per_word,
	(setter)SPIslave_set_bits_per_word,
	"Indicates the number of bits per one word. The typical numer of\n"
	"bits is 8, 16, 32. The default value is 8 bits.\n"},

	{"tx_actual_length",
	(getter)SPIslave_get_tx_actual_length,
	(setter)NULL, "When you want to monitore how much bytes is in\n"
		      "tx buffer.\n"},

	{"rx_actual_length",
	(getter)SPIslave_get_rx_actual_length,
	(setter)NULL, "When you want to monitore how much bytes is in\n"
		      "rx buffer."},

	{NULL},
};

static int SPIslave_init(SPIslave *self, PyObject *args)
{
	int device = -1;
	char path[MAX_PATH];

	if (!PyArg_ParseTuple(args, "i", &device))
		return -1;

	if (snprintf(path, MAX_PATH, "/dev/spislave%d", device) >= MAX_PATH) {
		PyErr_SetString(PyExc_OverflowError,
				"Device number is invalid.");
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
	SPIslave_doc,
	-1,
	NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC PyInit_SPIslave(void)
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
