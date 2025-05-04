#include "fir.h"

void __attribute__((section(".mprjram"))) initfir()
{
	// initial your fir
	for (int i = 0; i < N; i++)
	{
		inputbuffer[i] = 0;
		outputsignal[i] = 0;
	}
}

int *__attribute__((section(".mprjram"))) fir()
{
	initfir();
	// write down your fir

	for (int i = 0; i < N; i++)
	{
		int data_get = inputsignal[i]; // get data from axi-stream
		inputbuffer[i] = data_get;	   // store data to bram

		for (int j = 0; j <= i; j++)
		{
			outputsignal[i] += inputbuffer[j] * taps[i - j];
		}
	}
	return outputsignal;
}
