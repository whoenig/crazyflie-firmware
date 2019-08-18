/*
 * FILE: generated.h
 */

#ifndef __TAU_DIM6_SN_1_H__
#define __TAU_DIM6_SN_1_H__

/* Output */
typedef struct net_outputs2 {
	float out_0;
	float out_1;
	float out_2;
} net_outputs2;

/* The network */
void network2(net_outputs2 *net_out, float * state_array);

/* Computation of a single layer */
void layer2(float * in, 
            float * layer_weight, 
            float * layer_bias,   
            int row, 
            int col, 
            float * output, 
            int use_activation);

/* Sigmoid function */
float sigmoid2(float num);

/* Step function */
float step2(float num);

/* RELU function */
float relu2(float num);

/* Linear function */
float linear2(float num);

#endif
