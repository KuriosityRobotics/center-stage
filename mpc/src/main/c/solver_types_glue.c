//
// Created by Alec Petridis on 2023-07-28.
//

#include <jni.h>
#include "mecanum_mpc.h"
#include "jni_utils.h"

#include "solver_types_glue.h"

jobject construct_jsystem_state(JNIEnv *env, double const * state) {
    jclass systemStateClass = FIND_CLASS(env, "com/kuriosityrobotics/centerstage/mpc/SystemState");
    jobject systemState = (*env)->AllocObject(env, systemStateClass);

    (*env)->SetDoubleField(env, systemState, GET_FIELD_ID(env, systemStateClass, "fl", "D"), state[0]);
    (*env)->SetDoubleField(env, systemState, GET_FIELD_ID(env, systemStateClass, "fr", "D"), state[1]);
    (*env)->SetDoubleField(env, systemState, GET_FIELD_ID(env, systemStateClass, "bl", "D"), state[2]);
    (*env)->SetDoubleField(env, systemState, GET_FIELD_ID(env, systemStateClass, "br", "D"), state[3]);

    (*env)->SetDoubleField(env, systemState, GET_FIELD_ID(env, systemStateClass, "x", "D"), state[4]);
    (*env)->SetDoubleField(env, systemState, GET_FIELD_ID(env, systemStateClass, "y", "D"), state[5]);
    (*env)->SetDoubleField(env, systemState, GET_FIELD_ID(env, systemStateClass, "theta", "D"), state[6]);

    (*env)->SetDoubleField(env, systemState, GET_FIELD_ID(env, systemStateClass, "xVel", "D"), state[7]);
    (*env)->SetDoubleField(env, systemState, GET_FIELD_ID(env, systemStateClass, "yVel", "D"), state[8]);
    (*env)->SetDoubleField(env, systemState, GET_FIELD_ID(env, systemStateClass, "thetaVel", "D"), state[9]);

    return systemState;
}

jobjectArray construct_jsystem_state_array(JNIEnv *env, const mecanum_mpc_output *output) {
    jclass systemStateClass = FIND_CLASS(env, "com/kuriosityrobotics/centerstage/mpc/SystemState");

    int num_stages =  sizeof (*output) / sizeof (output->x1); // should be 5
    jobjectArray systemStates = (*env)->NewObjectArray(env, num_stages, systemStateClass, NULL);
    for (int i = 0; i < num_stages; i++) {
        jobjectArray currentSystemStateArray = construct_jsystem_state(env, (double const*)&(&output->x1)[i]);
        (*env)->SetObjectArrayElement(env, systemStates, i, currentSystemStateArray);

        (*env)->DeleteLocalRef(env, currentSystemStateArray);
    }
    return systemStates;
}

jobject construct_jsolver_output(JNIEnv *env, int exitCode, jobject solutionInfo, jobjectArray systemStates) {
    jclass solverOutputClass = FIND_CLASS(env, "com/kuriosityrobotics/centerstage/mpc/SolverOutput");

    jobject solverOutput = (*env)->AllocObject(env, solverOutputClass);
    (*env)->SetObjectField(env, solverOutput, GET_FIELD_ID(env, solverOutputClass, "states", "[Lcom/kuriosityrobotics/centerstage/mpc/SystemState;"), systemStates);
    (*env)->SetObjectField(env, solverOutput, GET_FIELD_ID(env, solverOutputClass, "solutionInfo", "Lcom/kuriosityrobotics/centerstage/mpc/SolutionInfo;"), solutionInfo);
    (*env)->SetIntField(env, solverOutput, GET_FIELD_ID(env, solverOutputClass, "exitCode", "I"), exitCode);
    return solverOutput;
}

jobject construct_jsolution_info(JNIEnv *env, const mecanum_mpc_info *info) {
    jclass solutionInfoClass = FIND_CLASS(env, "com/kuriosityrobotics/centerstage/mpc/SolutionInfo");
    jobject solutionInfo = (*env)->AllocObject(env, solutionInfoClass);

    (*env)->SetIntField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "it", "I"), info->it);
    (*env)->SetIntField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "it2opt", "I"), info->it2opt);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "resEq", "D"), info->res_eq);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "resIneq", "D"),
                           info->res_ineq);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "rsnorm", "D"), info->rsnorm);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "rcompnorm", "D"),
                           info->rcompnorm);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "pobj", "D"), info->pobj);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "dobj", "D"), info->dobj);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "dgap", "D"), info->dgap);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "rdgap", "D"), info->rdgap);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "mu", "D"), info->mu);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "muAff", "D"), info->mu_aff);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "sigma", "D"), info->sigma);
    (*env)->SetIntField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "lsitAff", "I"),
                        info->lsit_aff);
    (*env)->SetIntField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "lsitCc", "I"),
                        info->lsit_cc);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "stepAff", "D"),
                           info->step_aff);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "stepCc", "D"),
                           info->step_cc);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "solvetime", "D"),
                           info->solvetime);
    (*env)->SetDoubleField(env, solutionInfo, GET_FIELD_ID(env, solutionInfoClass, "fevalstime", "D"),
                           info->fevalstime);
    return solutionInfo;
}