#pragma once

#define FIND_CLASS(envVar, className) ({ \
    static jclass cls = NULL; \
    if (cls == NULL) { \
        cls = (*envVar)->FindClass(envVar, className); \
        if (cls != NULL) { \
            cls = (*envVar)->NewGlobalRef(envVar, cls); /* convert to global reference */ \
        }                                   \
    } \
    cls; \
})

#define GET_METHOD_ID(envVar, jclassVar, methodName, methodSig) ({ \
    static jmethodID mid = NULL; \
    if (mid == NULL) { \
        mid = (*envVar)->GetMethodID(envVar, jclassVar, methodName, methodSig); \
    } \
    mid; \
})

#define GET_STATIC_METHOD_ID(envVar, jclassVar, methodName, methodSig) ({ \
    static jmethodID mid = NULL; \
    if (mid == NULL) { \
        mid = (*envVar)->GetStaticMethodID(envVar, jclassVar, methodName, methodSig); \
    } \
    mid; \
})

#define GET_FIELD_ID(envVar, jclassVar, fieldName, fieldSig) ({ \
    static jfieldID fid = NULL; \
    if (fid == NULL) { \
        fid = (*envVar)->GetFieldID(envVar, jclassVar, fieldName, fieldSig); \
    } \
    fid; \
})
