// adapted from https://github.com/johnjwang/apriltag-android

#include <jni.h>
#include <opencv4/opencv2/core/mat.hpp>

#include <common/image_u8.h>

#include <iostream>

#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag36artoolkit.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"


static struct {
    apriltag_detector_t *td;
    apriltag_family_t *tf;
    void(*tf_destroy)(apriltag_family_t*);

    jclass al_cls;
    jmethodID al_constructor, al_add;
    jclass ad_cls;
    jmethodID ad_constructor;
    jfieldID ad_id_field, ad_hamming_field, ad_c_field, ad_p_field;
} state;

JNIEXPORT void JNICALL Java_edu_umich_eecs_april_apriltag_ApriltagNative_native_1init
    (JNIEnv *env, jclass cls)
{
    // Just do method lookups once and cache the results

    // Get ArrayList methods
    jclass al_cls = env->FindClass("java/util/ArrayList");
    if (!al_cls) {
        return;
    }
    state.al_cls = al_cls;

    state.al_constructor = env->GetMethodID(al_cls, "<init>", "()V");
    state.al_add = env->GetMethodID(al_cls, "add", "(Ljava/lang/Object;)Z");
    if (!state.al_constructor || !state.al_add) {
        return;
    }

    // Get ApriltagDetection methods
    jclass ad_cls = env->FindClass("edu/umich/eecs/april/apriltag/ApriltagDetection");
    if (!ad_cls) {
        
    }
    state.ad_cls = ad_cls;

    state.ad_constructor = env->GetMethodID(ad_cls, "<init>", "()V");
    if (!state.ad_constructor) {
        return;
    }

    state.ad_id_field = env->GetFieldID(ad_cls, "id", "I");
    state.ad_hamming_field = env->GetFieldID(ad_cls, "hamming", "I");
    state.ad_c_field = env->GetFieldID(ad_cls, "c", "[D");
    state.ad_p_field = env->GetFieldID(ad_cls, "p", "[D");
    if (!state.ad_id_field ||
            !state.ad_hamming_field ||
            !state.ad_c_field ||
            !state.ad_p_field) {
        return;
    }
}

/*
 * Class:     edu_umich_eecs_april_apriltag_ApriltagNative
 * Method:    yuv_to_rgb
 * Signature: ([BIILandroid/graphics/Bitmap;)V
 */
JNIEXPORT void JNICALL Java_edu_umich_eecs_april_apriltag_ApriltagNative_yuv_1to_1rgb
    (JNIEnv *env, jclass cls, jbyteArray _src, jint width, jint height, jlong dst_ptr)
{
    // NV21 Format
    // width*height    luma (Y) bytes followed by
    // width*height/2  chroma (UV) bytes interleaved as V,U

    jbyte *src = env->GetByteArrayElements(_src, NULL);

    if (!dst_ptr) {
        return;
    }

    cv::Mat* img_ptr = (cv::Mat*)dst_ptr;
    cv::Mat img(*img_ptr);

    image_u8_t img_header = { .width = img.cols,
    .height = img.rows,
    .stride = img.cols,
    .buf = img.data
};
    if (img_header.width*img_header.height != width*height) {
        std::cout << "incorrect bitmap format: %d x %d", img_header.width, img_header.height;
        return;
    }

    int uvstart = width * height;
    for (int j = 0; j < height; j += 1) {
        for (int i = 0; i < width; i += 1) {
            int y = (unsigned char)src[j*width + i];
            int offset = uvstart + (j >> 1)*width + (i & ~1);
            int u = (unsigned char)src[offset + 1];
            int v = (unsigned char)src[offset + 0];

            y = y < 16 ? 16 : y;

            int a0 = 1192 * (y - 16);
            int a1 = 1634 * (v - 128);
            int a2 = 832 * (v - 128);
            int a3 = 400 * (u - 128);
            int a4 = 2066 * (u - 128);

            int r = (a0 + a1) >> 10;
            int g = (a0 - a2 - a3) >> 10;
            int b = (a0 + a4) >> 10;

            r = r < 0 ? 0 : (r > 255 ? 255 : r);
            g = g < 0 ? 0 : (g > 255 ? 255 : g);
            b = b < 0 ? 0 : (b > 255 ? 255 : b);

            // Output image in portrait orientation
            img.at<cv::Vec3b>(i,j) = 0xff000000 | (r << 16) | (g << 8) | b;
        }
    }

    env->ReleaseByteArrayElements(_src, src, 0);
}

/*
 * Class:     edu_umich_eecs_april_apriltag_ApriltagNative
 * Method:    apriltag_init
 * Signature: (Ljava/lang/String;IDDI)V
 */
JNIEXPORT void JNICALL Java_edu_umich_eecs_april_apriltag_ApriltagNative_apriltag_1init
        (JNIEnv *env, jclass cls, jstring _tfname, jint errorbits, jdouble decimate,
         jdouble sigma, jint nthreads) {
    // Do cleanup in case we're already initialized
    if (state.td) {
        apriltag_detector_destroy(state.td);
        state.td = NULL;
    }
    if (state.tf) {
        state.tf_destroy(state.tf);
        state.tf = NULL;
    }

    // Initialize state
    const char *tfname = env->GetStringUTFChars(_tfname, NULL);

    if (!strcmp(tfname, "tag36h11")) {
        state.tf = tag36h11_create();
        state.tf_destroy = tag36h11_destroy;
    } else if (!strcmp(tfname, "tag36h10")) {
        state.tf = tag36h10_create();
        state.tf_destroy = tag36h10_destroy;
    } else if (!strcmp(tfname, "tag36artoolkit")) {
        state.tf = tag36artoolkit_create();
        state.tf_destroy = tag36artoolkit_destroy;
    } else if (!strcmp(tfname, "tag25h9")) {
        state.tf = tag25h9_create();
        state.tf_destroy = tag25h9_destroy;
    } else if (!strcmp(tfname, "tag25h7")) {
        state.tf = tag25h7_create();
        state.tf_destroy = tag25h7_destroy;
    } else if (!strcmp(tfname, "tag16h5")) {
        state.tf = tag16h5_create();
        state.tf_destroy = tag16h5_destroy;
    } else {
        std::cout << "invalid tag family: %s", tfname;
        env->ReleaseStringUTFChars(_tfname, tfname);
        return;
    }
    env->ReleaseStringUTFChars(_tfname, tfname);

    state.td = apriltag_detector_create();
    apriltag_detector_add_family_bits(state.td, state.tf, errorbits);
    state.td->quad_decimate = decimate;
    state.td->quad_sigma = sigma;
    state.td->nthreads = nthreads;
}

/*
 * Class:     edu_umich_eecs_april_apriltag_ApriltagNative
 * Method:    apriltag_detect_yuv
 * Signature: ([BII)Ljava/util/ArrayList;
 */
JNIEXPORT jobject JNICALL Java_edu_umich_eecs_april_apriltag_ApriltagNative_apriltag_1detect_1yuv
        (JNIEnv *env, jclass cls, jbyteArray _buf, jint width, jint height) {
    // If not initialized, init with default settings
    if (!state.td) {
        state.tf = tag36h11_create();
        state.td = apriltag_detector_create();
        apriltag_detector_add_family_bits(state.td, state.tf, 2);
        state.td->quad_decimate = 2.0;
        state.td->quad_sigma = 0.0;
        state.td->nthreads = 4;
        std::cout << "using default parameters";
    }

    // Use the luma channel (the first width*height elements)
    // as grayscale input image
    jbyte *buf = env->GetByteArrayElements(_buf, NULL);
    image_u8_t im = {
            .width = width,
            .height = height,  
            .stride = width,
            .buf = (uint8_t*)buf
    };
    zarray_t *detections = apriltag_detector_detect(state.td, &im);
    env->ReleaseByteArrayElements(_buf, buf, 0);

    // al = new ArrayList();
    jobject al = env-> NewObject(state.al_cls, state.al_constructor);
    for (int i = 0; i < zarray_size(detections); i += 1) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // ad = new ApriltagDetection();
        jobject ad = env->NewObject(state.ad_cls, state.ad_constructor);
        env->SetIntField(ad, state.ad_id_field, det->id);
        env->SetIntField(ad, state.ad_hamming_field, det->hamming);
        jobject ob_ad_c = env->GetObjectField(ad, state.ad_c_field);
        jdoubleArray* ad_c_ptr = reinterpret_cast<jdoubleArray*>(&ob_ad_c);
        jdoubleArray ad_c(*ad_c_ptr);
        env->SetDoubleArrayRegion(ad_c, 0, 2, det->c);
        jobject ob_ad_p = env->GetObjectField(ad, state.ad_p_field);
        jdoubleArray* ad_p_ptr = reinterpret_cast<jdoubleArray*>(&ob_ad_c);
        jdoubleArray ad_p(*ad_p_ptr);
        env->SetDoubleArrayRegion(ad_p, 0, 8, (double*)det->p);

        // al.add(ad);
        env->CallBooleanMethod(al, state.al_add, ad);

        // Need to respect the local reference limit
        env->DeleteLocalRef(ad);
        env->DeleteLocalRef(ad_c);
        env->DeleteLocalRef(ad_p);
    }

    // Cleanup
    apriltag_detections_destroy(detections);

    return al;
}
