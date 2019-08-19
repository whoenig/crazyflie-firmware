/*
 * FILE: generated.h
 */

#ifndef __THREE_DIM6_SN_1_H__
#define __THREE_DIM6_SN_1_H__

/* Output */
typedef struct net_outputs {
	float out_0;
} net_outputs;

/* The network */
void network(net_outputs *net_out, float * state_array);

/* Computation of a single layer */
void layer(float * in, 
            float * layer_weight, 
            float * layer_bias,   
            int row, 
            int col, 
            float * output, 
            int use_activation);

/* Sigmoid function */
float sigmoid(float num);

/* Step function */
float step(float num);

/* RELU function */
float relu(float num);

/* Linear function */
float linear(float num);

#endif
