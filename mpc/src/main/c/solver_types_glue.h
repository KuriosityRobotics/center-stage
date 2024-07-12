//
// Created by Alec Petridis on 2023-07-28.
//

#pragma once

#include <jni.h>
#include "mecanum_mpc.h"

jobjectArray construct_jsystem_state_array(JNIEnv *env, const mecanum_mpc_output *output);
jobject construct_jsystem_state(JNIEnv *env, double const* state);

jobject construct_jsolver_output(JNIEnv *env, int exitCode, jobject solutionInfo, jobjectArray systemStates);
jobject construct_jsolution_info(JNIEnv *env, const mecanum_mpc_info *info);