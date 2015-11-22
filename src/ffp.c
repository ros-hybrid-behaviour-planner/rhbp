/*
 ============================================================================
 Name        : c-python-test.c
 Author      : Stephan Wypler
 Version     :
 Copyright   : 
 Description : Hello World in C, Ansi-style
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

static PyObject* plan(PyObject* self, PyObject* args, PyObject* kw){
	int pipefd[2];
    if(pipe(pipefd)!= 0){ // create the pipe
    	PyErr_SetString(PlannerProxyError, strerror(errno));
    	return NULL;
    }
    fflush(stdout); // this is actually just cosmetics
	pid_t pid = fork();
	if(pid == 0){
		close(pipefd[0]); // close the read-end of the pipe, I'm not going to use it
		void *handle;
		PyObject* (*ff_plan)(PyObject*, PyObject*, PyObject*);
		char *error;
		handle = dlopen("ff.so", RTLD_LAZY | RTLD_LOCAL);
		if (!handle) {
			puts(dlerror());
			close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
			exit(EXIT_FAILURE);
		}
		dlerror();    /* Clear any existing error */

		/* Writing: The C99 standard leaves
		       casting from "void *" to a function pointer undefined.
		       The assignment used below is the POSIX.1-2003 (Technical
		       Corrigendum 1) workaround; see the Rationale for the
		       POSIX specification of dlsym(). */
		*(void **) (&ff_plan) = dlsym(handle, "ff_plan");

		if ((error = dlerror()) != NULL)  {
			puts(error);
			close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
			exit(EXIT_FAILURE);
		}

		PyObject* plannerResult = (*ff_plan)(self, args, kw);
		if(dlclose(handle) != 0){
			puts(dlerror());
			close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
			exit(EXIT_FAILURE);
		}
		if(!plannerResult){
			PyErr_Print();
			close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
			exit(EXIT_FAILURE);
		}
		PyObject* serialisedResult = PyMarshal_WriteObjectToString(plannerResult, Py_MARSHAL_VERSION);
		Py_DECREF(plannerResult);
		if(!serialisedResult){
			puts("Failed to marshal result");
			close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
			exit(EXIT_FAILURE);
		}
		Py_ssize_t len = PyString_Size(serialisedResult);
		size_t totalBytesWritten = 0;
		size_t currentBytesWritten = 0;
		while(totalBytesWritten < sizeof(Py_ssize_t)){
			if((currentBytesWritten = write(pipefd[1], (char*) &len + totalBytesWritten, sizeof(Py_ssize_t) - totalBytesWritten)) != -1)
				totalBytesWritten += currentBytesWritten;
			else{
				puts("Failed to communicate result size to parent process");
				close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
				exit(EXIT_FAILURE);
			}
		}
		totalBytesWritten = 0;
		currentBytesWritten = 0;
		while(totalBytesWritten < len){
			if((currentBytesWritten = write(pipefd[1], PyString_AsString(serialisedResult) + totalBytesWritten, len - totalBytesWritten)) != -1)
				totalBytesWritten += currentBytesWritten;
			else{
				puts("Failed to communicate result to parent process");
				close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
				exit(EXIT_FAILURE);
			}
		}
		Py_DECREF(serialisedResult);
		close(pipefd[1]); // close the write-end of the pipe, thus sending EOF
        exit(EXIT_SUCCESS);
	} else if(pid > 0){
		close(pipefd[1]); // close the write-end of the pipe, I'm not going to use it
		Py_ssize_t len;
		size_t totalBytesRead = 0;
		size_t currentBytesRead = 0;
		while(totalBytesRead < sizeof(Py_ssize_t)){
			if((currentBytesRead = read(pipefd[0], (char*) &len + totalBytesRead, sizeof(Py_ssize_t) - totalBytesRead)) > 0)
				totalBytesRead += currentBytesRead;
			else{
                waitpid(pid, NULL, WNOHANG);
				PyErr_SetString(PlannerProxyError, "Failed to read result size from child");
				return NULL;
			}
		}
		char* serialisedResult = (char*) calloc(len, sizeof(char));
		if(!serialisedResult)
			return PyErr_NoMemory();
		totalBytesRead = 0;
		currentBytesRead = 0;
		while(totalBytesRead < len){
			if((currentBytesRead = read(pipefd[0], serialisedResult + totalBytesRead, len - totalBytesRead)) > 0)
				totalBytesRead += currentBytesRead;
			else{
                waitpid(pid, NULL, WNOHANG);
				PyErr_SetString(PlannerProxyError, "Failed to read result from child");
				return NULL;
			}
		}
		close(pipefd[0]); // close the read-end of the pipe
		waitpid(pid, NULL, 0);
		PyObject* result = PyMarshal_ReadObjectFromString(serialisedResult, len);
		free(serialisedResult);
		return result;
	} else {
        PyErr_SetString(PlannerProxyError, strerror(errno));
		return NULL;
	}
}

static PyMethodDef PlannerProxyMethods[] = {
		{"plan",  plan, METH_VARARGS | METH_KEYWORDS, "Do the planner process spawning magic."},
		{NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC initffp(void){
	PyObject *m;

	m = Py_InitModule("ffp", PlannerProxyMethods);
	if (m == NULL)
		return;

	PlannerProxyError = PyErr_NewException("ffp.error", NULL, NULL);
	Py_INCREF(PlannerProxyError);
	PyModule_AddObject(m, "error", PlannerProxyError);
}

