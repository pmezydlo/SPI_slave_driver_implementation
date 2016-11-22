#include <Python.h>

static PyObject*
SPIslave_open(PyObject* self, PyObject* args)
{
    const int bus;

    if (!PyArg_ParseTuple(args, "d", &bus))
        return NULL;

    printf("spislave %d\n", bus);

    Py_RETURN_NONE;
}

static PyObject*
SPIslave_close(PyObject* self, PyObject* args)
{

}

static PyObject*
SPIslave_close(PyObject* self, PyObject* args)
{

}


static PyMethodDef SPIslave_Methods[] =
{
     {"open", SPIslave_open, METH_VARARGS, "open desc"},
     {"close", SPIslave_close, METH_VARARGS, "close desc"},
     {"read", SPIslave_read, METH_VARARGS, "read desc"},
     {"write", SPIslave_write, METH_VRARGS, "write desc"},
     {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC
initSPIslave(void)
{
	(void) Py_InitModule("SPIslave", SPIslave_Methods);
}
