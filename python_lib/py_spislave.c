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

static PyMethodDef SPIslave_Methods[] =
{
     {"open", SPIslave_open, METH_VARARGS, "open desc"},
     {NULL, NULL, 0, NULL}
};

PyMODINIT_FUNC
initSPIslave(void)
{
     (void) Py_InitModule("SPIslave", SPIslave_Methods);
}
