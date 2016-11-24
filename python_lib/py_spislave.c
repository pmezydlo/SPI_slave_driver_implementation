#include <Python.h>

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

	printf("spi slave new\n");

	self->fd = -1;
	self->mode = 0;
	self->tx_actual_length = 0;
	self->rx_actual_length = 0;
	self->max_speed = 0;
	self->bits_per_word = 0;

	return (PyObject *)self;
}


static PyObject*
SPIslave_open(SPIslave* self, PyObject* args, PyObject kwds)
{
    const int bus;

    if (!PyArg_ParseTuple(args, "d", &bus))
        return NULL;

    printf("spislave %d\n", bus);

    Py_RETURN_NONE;
}

static PyObject*
SPIslave_close(SPIslave* self)
{


	Py_RETURN_NONE;
}

static void
SPIslave_dealloc(SPIslave *self)
{
	PyObject *ref = SPIslave_close(self);
	Py_XDECREF(ref);
	Py_TYPE(self)->tp_free((PyObject *)self);
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

static PyObject*
SPIslave_get_mode(SPIslave *self, void *closure)
{
	PyObject *result = NULL;/*now is null dont use it*/

	return result;
}

static int
SPIslave_set_mode(SPIslave *self, PyObject *val, void *closure)
{


	return 0;
}

static PyGetSetDef SPIslave_getset[] = {
	{"mode", (getter)SPIslave_get_mode, (setter)SPIslave_set_mode, "spislave mode"},
	{NULL},
};

static int
SPIslave_init(SPIslave *self, PyObject *args, PyObject *kwds)
{

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
initSPIslave(void)
{
	PyObject *m;

	SPIslave_type.tp_new = PyType_GenericNew;
	if (PyType_Ready(&SPIslave_type) < 0)
		return NULL;

	m = PyModule_Create(&SPIslave_module);
	if (m==NULL)
		return NULL;

	Py_INCREF(&SPIslave_type);
	PyModule_AddObject(m, "SPIslave", (PyObject *)&SPIslave_type);

	return m;
}
