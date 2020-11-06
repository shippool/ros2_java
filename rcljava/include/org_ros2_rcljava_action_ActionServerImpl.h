// Copyright 2020 ros2-java contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <jni.h>
/* Header for class org_ros2_rcljava_action_ActionServerImpl */

#ifndef ORG_ROS2_RCLJAVA_ACTION_ACTIONSERVERIMPL_H_
#define ORG_ROS2_RCLJAVA_ACTION_ACTIONSERVERIMPL_H_
#ifdef __cplusplus
extern "C" {
#endif

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeGetNumberOfSubscriptions
 * Signature: (L)I
 */
JNIEXPORT jint
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeGetNumberOfSubscriptions(
  JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeGetNumberOfTimers
 * Signature: (L)I
 */
JNIEXPORT jint
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeGetNumberOfTimers(
  JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeGetNumberOfClients
 * Signature: (L)I
 */
JNIEXPORT jint
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeGetNumberOfClients(
  JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeGetNumberOfServices
 * Signature: (L)I
 */
JNIEXPORT jint
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeGetNumberOfServices(
  JNIEnv *, jclass, jlong);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeGetReadyEntities
 * Signature: (LL)[Z
 */
JNIEXPORT jbooleanArray
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeGetReadyEntities(
  JNIEnv *, jclass, jlong, jlong);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeDispose
 * Signature: (JJ)V
 */
JNIEXPORT void
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeDispose(JNIEnv *, jclass, jlong, jlong);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeCreateActionServer
 * Signature: (JLjava/lang/Class;Ljava/lang/String;)J
 */
JNIEXPORT jlong
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeCreateActionServer(
  JNIEnv *, jobject, jlong, jlong, jclass, jstring);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeTakeGoalRequest
 * Signature: (JJJJLorg/ros2/rcljava/interfaces/MessageDefinition;)Lorg/ros2/rcljava/RMWRequestId;
 */
JNIEXPORT jobject
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeTakeGoalRequest(
  JNIEnv *, jclass, jlong, jlong, jlong, jlong, jobject);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeTakeCancelRequest
 * Signature: (JJJJLorg/ros2/rcljava/interfaces/MessageDefinition;)Lorg/ros2/rcljava/RMWRequestId;
 */
JNIEXPORT jobject
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeTakeCancelRequest(
  JNIEnv *, jclass, jlong, jlong, jlong, jlong, jobject);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeTakeResultRequest
 * Signature: (JJJJLorg/ros2/rcljava/interfaces/MessageDefinition;)Lorg/ros2/rcljava/RMWRequestId;
 */
JNIEXPORT jobject
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeTakeResultRequest(
  JNIEnv *, jclass, jlong, jlong, jlong, jlong, jobject);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeSendGoalResponse
 * Signature: (JLorg/ros2/rcljava/RMWRequestId;JJJLorg/ros2/rcljava/interfaces/MessageDefinition;)
 */
JNIEXPORT void
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeSendGoalResponse(
  JNIEnv *, jclass, jlong, jobject, jlong, jlong, jlong, jobject);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeSendCancelResponse
 * Signature: (JLorg/ros2/rcljava/RMWRequestId;JJJLorg/ros2/rcljava/interfaces/MessageDefinition;)
 */
JNIEXPORT void
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeSendCancelResponse(
  JNIEnv *, jclass, jlong, jobject, jlong, jlong, jlong, jobject);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeSendResultResponse
 * Signature: (JLorg/ros2/rcljava/RMWRequestId;JJJLorg/ros2/rcljava/interfaces/MessageDefinition;)
 */
JNIEXPORT void
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeSendResultResponse(
  JNIEnv *, jclass, jlong, jobject, jlong, jlong, jlong, jobject);

/*
 * Class:     org_ros2_rcljava_action_ActionServerImpl
 * Method:    nativeProcessCancelRequest
 * Signature: (JJJJJJJLorg/ros2/rcljava/interfaces/MessageDefinition;Lorg/ros2/rcljava/interfaces/MessageDefinition;)
 */
JNIEXPORT void
JNICALL Java_org_ros2_rcljava_action_ActionServerImpl_nativeProcessCancelRequest(
  JNIEnv *, jclass, jlong, jlong, jlong, jlong, jlong, jlong, jlong, jobject, jobject);

#ifdef __cplusplus
}
#endif
#endif  // ORG_ROS2_RCLJAVA_ACTION_ACTIONSERVERIMPL_H__
