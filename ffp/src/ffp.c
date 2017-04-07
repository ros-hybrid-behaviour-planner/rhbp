/*
 ============================================================================
 Name        : ffp.c
 Author      : Stephan Wypler
 Version     :
 Copyright   : 
 Description : Python extension module providing a python interface for metric ff planner
 ============================================================================
 */
#include <sys/wait.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <python2.7/Python.h>
#include <python2.7/marshal.h>

static PyObject* PlannerProxyError;

static PyObject* plan(PyObject* self, PyObject* args, PyObject* kw)
{
  //create process pipe
  int pipefd[2];

  if (pipe(pipefd) != 0)
  { // create the pipe
    PyErr_SetString(PlannerProxyError, strerror(errno));
    return NULL;
  }

  fflush(stdout); // this is actually just cosmetics
  pid_t pid = fork();
  if (pid == 0)
  { //The process child part
    close(pipefd[0]); // close the read-end of the pipe, I'm not going to use it

    void *handle;
    PyObject* (*ff_plan)(PyObject*, PyObject*, PyObject*);
    void (*initff)(void);
    char *error;
    handle = dlopen("ff.so", RTLD_LAZY | RTLD_GLOBAL);
    if (!handle)
    {
      puts(dlerror());
      close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
      exit(EXIT_FAILURE);
    }
    dlerror(); /* Clear any existing error */

    /* Writing: The C99 standard leaves
     casting from "void *" to a function pointer undefined.
     The assignment used below is the POSIX.1-2003 (Technical
     Corrigendum 1) workaround; see the Rationale for the
     POSIX specification of dlsym(). */
    *(void **)(&initff) = dlsym(handle, "initff");
    if ((error = dlerror()) != NULL)
    {
      puts(error);
      close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
      exit(EXIT_FAILURE);
    }

    *(void **)(&ff_plan) = dlsym(handle, "ff_plan");

    if ((error = dlerror()) != NULL)
    {
      puts(error);
      close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
      exit(EXIT_FAILURE);
    }
    //first init dll for exceptions registration etc.
    (*initff)();

    //call the planning function
    PyObject* plannerResult = (*ff_plan)(self, args, kw);

    if (dlclose(handle) != 0)
    {
      puts(dlerror());
      close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
      exit(EXIT_FAILURE);
    }

    if (PyErr_Occurred()) {
      PyErr_Print();
      close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
      exit(EXIT_FAILURE);
    }

    if (!plannerResult)
    {
      puts("ERROR: Planning result empty");
      close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
      exit(EXIT_FAILURE);
    }

    PyObject* serialisedResult = PyMarshal_WriteObjectToString(plannerResult, Py_MARSHAL_VERSION);
    Py_DECREF(plannerResult);
    if (!serialisedResult)
    {
      puts("ERROR: Failed to marshal result");
      close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
      exit(EXIT_FAILURE);
    }

    Py_ssize_t len = PyString_Size(serialisedResult);
    size_t totalBytesWritten = 0;
    size_t currentBytesWritten = 0;
    while (totalBytesWritten < sizeof(Py_ssize_t))
    {
      if ((currentBytesWritten = write(pipefd[1], (char*)&len + totalBytesWritten,
                                       sizeof(Py_ssize_t) - totalBytesWritten)) != -1)
        totalBytesWritten += currentBytesWritten;
      else
      {
        puts("ERROR: Failed to communicate result size to parent process");
        close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
        exit(EXIT_FAILURE);
      }
    }

    totalBytesWritten = 0;
    currentBytesWritten = 0;
    while (totalBytesWritten < len)
    {
      if ((currentBytesWritten = write(pipefd[1], PyString_AsString(serialisedResult) + totalBytesWritten,
                                       len - totalBytesWritten)) != -1)
        totalBytesWritten += currentBytesWritten;
      else
      {
        puts("ERROR: Failed to communicate result to parent process");
        close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
        exit(EXIT_FAILURE);
      }
    }
    Py_DECREF(serialisedResult);
    close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
    exit(EXIT_SUCCESS);
  }
  else if (pid > 0)
  { //the process parent part
    close(pipefd[1]); // close the write-end of the pipe, it is not going to be used

    PyObject* ret = -1;

    Py_ssize_t len;
    size_t totalBytesRead = 0;
    size_t currentBytesRead = 0;
    while (totalBytesRead < sizeof(Py_ssize_t))
    {
      if ((currentBytesRead = read(pipefd[0], (char*)&len + totalBytesRead, sizeof(Py_ssize_t) - totalBytesRead)) > 0)
        totalBytesRead += currentBytesRead;
      else
      {
        int status = 0;
        waitpid(pid, &status, WUNTRACED);
        /*
         * Although not reading a result size means that planning went wrong there are two expected cases of this happening:
         * The first is when the planner exited 0x42 which means that all goals are already satisfied and there is nothing to do.
         * The second case is that the problem cannot be solved because it is provably impossible. In this case the FF exits with 0xFF.
         */
        if (WIFEXITED(status))
        { // if the child exited normally and did not crash
          PyObject* result = PyDict_New();
          if (!result)
          {
            ret = NULL;
            PyErr_SetString(PlannerProxyError, "Couldn't create python dict");
            break;
          }
          PyObject* actions = PyDict_New();
          if (!actions)
          {
            Py_DecRef(result);
            PyErr_SetString(PlannerProxyError, "Couldn't create python dict");
            ret = NULL;
            break;
          }
          if (PyDict_SetItemString(result, "actions", actions) != 0)
          {
            Py_DecRef(result);
            Py_DecRef(actions);
            PyErr_SetString(PlannerProxyError, "Couldn't set actions string");
            ret = NULL;
            break;
          }
          Py_DecRef(actions);
          if (WEXITSTATUS(status) == 0x42)
          { // all goals are already fulfilled
            PyObject* cost_value = PyFloat_FromDouble(0);
            if (!cost_value)
            {
              Py_DecRef(result);
              PyErr_SetString(PlannerProxyError, "Couldn't create costs float");
              ret = NULL;
              break;
            }
            if (PyDict_SetItemString(result, "cost", cost_value) != 0)
            {
              Py_DecRef(result);
              PyErr_SetString(PlannerProxyError, "Couldn't set costs");
              ret = NULL;
              break;
            }
            Py_DecRef(cost_value);
            ret = result;
          }
          else if (WEXITSTATUS(status) == 0xFF)
          { // plan not solvable
            PyObject* cost_value = PyFloat_FromDouble(-1); // This is the indicator for python the planning problem is impossible
            if (!cost_value)
            {
              PyErr_SetString(PlannerProxyError, "Couldn't create costs float");
              Py_DecRef(result);
              ret = NULL;
              break;
            }
            if (PyDict_SetItemString(result, "cost", cost_value) != 0)
            {
              Py_DecRef(result);
              PyErr_SetString(PlannerProxyError, "Couldn't set costs");
              ret = NULL;
              break;
            }
            Py_DecRef(cost_value);
            ret = result;
          }
          else if (WEXITSTATUS(status) == EXIT_FAILURE)
          {
            Py_DecRef(result);
            PyErr_SetString(PlannerProxyError, "Planner exited with failure.");
            ret = NULL;
            break;
          }
          else if (WEXITSTATUS(status) == EXIT_SUCCESS)
          {
            //this might occur with an empty plan which was simplified to false
            Py_DecRef(result);
            break;
          }
          else
          {
            Py_DecRef(result);
            char msg[100];
            snprintf(msg, sizeof(msg), "Unexpected exit status: %x", WEXITSTATUS(status));
            PyErr_SetString(PlannerProxyError, msg);
            ret = NULL;
            break;
          }
        }else{
          char msg[100];
          snprintf(msg, sizeof(msg), "Planner exited abnormally. Exit status: %x\n", WEXITSTATUS(status));
          PyErr_SetString(PlannerProxyError, msg);
          ret = NULL;
          break;
        }
      }
    }
    //ret == 1 means that no ret value was assigned before, which would indicate some error handling before
    //thus we continue with default result reading and deserialization
    if(ret == -1)
    {
      char* serialisedResult = (char*)calloc(len, sizeof(char));
      if (!serialisedResult)
      {
        close(pipefd[0]);
        return PyErr_NoMemory();
      }
      totalBytesRead = 0;
      currentBytesRead = 0;
      while (totalBytesRead < len)
      {
        if ((currentBytesRead = read(pipefd[0], serialisedResult + totalBytesRead, len - totalBytesRead)) > 0)
          totalBytesRead += currentBytesRead;
        else
        {
          waitpid(pid, NULL, WNOHANG);
          PyErr_SetString(PlannerProxyError, "Failed to read result from child");
          ret = NULL;
        }
      }
      close(pipefd[0]); // close the read-end of the pipe
      waitpid(pid, NULL, 0);

      PyObject* result = PyMarshal_ReadObjectFromString(serialisedResult, len);
      free(serialisedResult);
      return result;
    }else
    {
      close(pipefd[0]); // close the read-end of the pipe
      waitpid(pid, NULL, 0);
	  return ret;
    }
  }
  else
  {
    PyErr_SetString(PlannerProxyError, strerror(errno));
    return NULL;
  }
}

static PyMethodDef PlannerProxyMethods[] = { {"plan", plan, METH_VARARGS | METH_KEYWORDS,
                                              "Do the planner process spawning magic."},
                                            {NULL, NULL, 0, NULL} /* Sentinel */
};

PyMODINIT_FUNC initffp(void)
{
  PyObject *m;

  m = Py_InitModule("ffp", PlannerProxyMethods);
  if (m == NULL)
  {
    return;
  }

  PlannerProxyError = PyErr_NewException("ffp.error", NULL, NULL);
  Py_INCREF(PlannerProxyError);
  PyModule_AddObject(m, "error", PlannerProxyError);
}

