//
// Created by Max Cai on 7/11/23.
//

#pragma once

#include <jni.h>

#include "mecanum_mpc.h"

JNIEXPORT jobject JNICALL Java_com_kuriosityrobotics_centerstage_mpc_SolverInput_solve
        (JNIEnv *env, jobject thisObject);