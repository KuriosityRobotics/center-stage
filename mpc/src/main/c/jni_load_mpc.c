//
// Created by Max Cai on 7/11/23.
//

#include <stdbool.h>
#include <string.h>

#include "jni_load_mpc.h"
#include "jni_utils.h"
#include "solver_types_glue.h"
#include "mecanum_mpc_memory.h"

// printing includes
#include <stdio.h>

JNIEXPORT jobject JNICALL Java_com_kuriosityrobotics_centerstage_mpc_SolverInput_solve(JNIEnv *env, jobject thisObject) {
    jclass solverInputClass = FIND_CLASS(env, "com/kuriosityrobotics/centerstage/mpc/SolverInput");

    jmethodID getStateArray = GET_METHOD_ID(env, solverInputClass, "getStateArray", "()[D");
    jmethodID getParamsArray = GET_METHOD_ID(env, solverInputClass, "getOptimisationParametersArray", "()[D");
    jmethodID getGuessesArray = GET_METHOD_ID(env, solverInputClass, "getInitialGuessesArray", "()[D");

    jdoubleArray stateArray = (*env)->CallObjectMethod(env, thisObject, getStateArray);
    jdoubleArray paramsArray = (*env)->CallObjectMethod(env, thisObject, getParamsArray);
    jdoubleArray guessesArray = (*env)->CallObjectMethod(env, thisObject, getGuessesArray);

    mecanum_mpc_params mpcParams;

    double* x0 = (*env)->GetDoubleArrayElements(env, guessesArray, false);
    double* xinit = (*env)->GetDoubleArrayElements(env, stateArray, false);
    double* all_parameters = (*env)->GetDoubleArrayElements(env, paramsArray, false);

    memcpy(mpcParams.x0, x0, sizeof(double) * 50);
    memcpy(mpcParams.xinit, xinit, sizeof(double) * 6);
    memcpy(mpcParams.all_parameters, all_parameters, sizeof(double) * 160);

    (*env)->ReleaseDoubleArrayElements(env, guessesArray, x0, JNI_ABORT);
    (*env)->ReleaseDoubleArrayElements(env, stateArray, xinit, JNI_ABORT);
    (*env)->ReleaseDoubleArrayElements(env, paramsArray, all_parameters, JNI_ABORT);

    (*env)->DeleteLocalRef(env, stateArray);
    (*env)->DeleteLocalRef(env, guessesArray);
    (*env)->DeleteLocalRef(env, paramsArray);

    mecanum_mpc_output output;
    mecanum_mpc_info info;
    mecanum_mpc_mem *args = mecanum_mpc_internal_mem(0);
    FILE *debug_output = stdout;

    memset(&output, 0, sizeof(mecanum_mpc_output));
    memset(&info, 0, sizeof(mecanum_mpc_info));

    int exitCode = mecanum_mpc_solve(&mpcParams, &output, &info, args, debug_output, mecanum_mpc_adtool2forces); // juicy

    jobject solutionInfo = construct_jsolution_info(env, &info);
    jobjectArray systemStates = construct_jsystem_state_array(env, &output);

    jobject solverOutput = construct_jsolver_output(env, exitCode, solutionInfo, systemStates);

    return solverOutput;
}
