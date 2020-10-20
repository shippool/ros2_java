/* Copyright 2020 ros2-java contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.ros2.rcljava.action;

import java.lang.ref.WeakReference;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.common.JNIUtils;
import org.ros2.rcljava.consumers.Consumer;
import org.ros2.rcljava.interfaces.MessageDefinition;
import org.ros2.rcljava.interfaces.ActionDefinition;
import org.ros2.rcljava.interfaces.GoalRequestDefinition;
import org.ros2.rcljava.interfaces.GoalResponseDefinition;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.service.RMWRequestId;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ActionServerImpl<T extends ActionDefinition> implements ActionServer<T> {
  private static final Logger logger = LoggerFactory.getLogger(ActionServerImpl.class);

  static {
    try {
      JNIUtils.loadImplementation(ActionServerImpl.class);
    } catch (UnsatisfiedLinkError ule) {
      logger.error("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  private final WeakReference<Node> nodeReference;
  private final T actionTypeInstance;
  private final String actionName;
  private long handle;
  private final GoalCallback<? extends GoalRequestDefinition> goalCallback;
  private final CancelCallback<T> cancelCallback;
  private final Consumer<ActionServerGoalHandle<T>> acceptedCallback;

  private boolean[] readyEntities;

  private boolean isGoalRequestReady() {
    return this.readyEntities[0];
  }

  private boolean isCancelRequestReady() {
    return this.readyEntities[1];
  }

  private boolean isResultRequestReady() {
    return this.readyEntities[2];
  }

  private boolean isGoalExpiredReady() {
    return this.readyEntities[3];
  }

  private native long nativeCreateActionServer(
    long nodeHandle, long clockHandle, Class<T> cls, String actionName);

  /**
   * Create an action server.
   *
   * @param nodeReference A reference to the node to use to create this action server.
   * @param actionType The type of the action.
   * @param actionName The name of the action.
   * @param goalCallback Callback triggered when a new goal request is received.
   * @param cancelCallback Callback triggered when a new cancel request is received.
   * @param acceptedCallback Callback triggered when a new goal is accepted.
   */
  public ActionServerImpl(
      final WeakReference<Node> nodeReference,
      final Class<T> actionType,
      final String actionName,
      final GoalCallback<? extends GoalRequestDefinition> goalCallback,
      final CancelCallback<T> cancelCallback,
      final Consumer<ActionServerGoalHandle<T>> acceptedCallback) throws IllegalArgumentException {
    this.nodeReference = nodeReference;
    try {
      this.actionTypeInstance = actionType.newInstance();
    } catch (Exception ex) {
      throw new IllegalArgumentException("Failed to instantiate provided action type", ex);
    }
    this.actionName = actionName;
    this.goalCallback = goalCallback;
    this.cancelCallback = cancelCallback;
    this.acceptedCallback = acceptedCallback;

    Node node = nodeReference.get();
    if (node == null) {
      throw new IllegalArgumentException("Node reference is null");
    }

    this.handle = nativeCreateActionServer(
      node.getHandle(), node.getClock().getHandle(), actionType, actionName);
    // TODO(jacobperron): Introduce 'Waitable' interface for entities like timers, services, etc
    // node.addWaitable(this);
  }

  private static native int nativeGetNumberOfSubscriptions();
  private static native int nativeGetNumberOfTimers();
  private static native int nativeGetNumberOfClients();
  private static native int nativeGetNumberOfServices();

  /**
   * {@inheritDoc}
   */
  public int getNumberOfSubscriptions() {
    return nativeGetNumberOfSubscriptions();
  }

  /**
   * {@inheritDoc}
   */
  public int getNumberOfTimers() {
    return nativeGetNumberOfTimers();
  }

  /**
   * {@inheritDoc}
   */
  public int getNumberOfClients() {
    return nativeGetNumberOfClients();
  }

  /**
   * {@inheritDoc}
   */
  public int getNumberOfServices() {
    return nativeGetNumberOfServices();
  }

  private static native boolean[] nativeGetReadyEntities(
    long actionServerHandle, long waitSetHandle);

  /**
   * {@inheritDoc}
   */
  public boolean isReady(long waitSetHandle) {
    this.readyEntities = nativeGetReadyEntities(this.handle, waitSetHandle);
    for (boolean isReady : this.readyEntities) {
      if (isReady) {
        return true;
      }
    }
    return false;
  }

  private void executeGoalCallback(
    RMWRequestId rmwRequestId,
    GoalRequestDefinition requestMessage,
    GoalResponseDefinition responseMessage)
  {
    // Call user callback
    // Workaround type
    GoalCallback<GoalRequestDefinition> callback = ((ActionServerImpl) this).goalCallback;
    GoalResponse response = callback.handleGoal(requestMessage);

    boolean accepted = GoalResponse.ACCEPT == response;

    // Populate response
    // TODO
    // response.setStamp(final builtin_interfaces.msg.Time stamp);
    responseMessage.accept(accepted);

    // If accepted, then create a goal handle and schedule user accepted callback
    // ActionServerGoalHandle<T> goalHandle = new ActionServerGoalHandle<T>(requestMessage.getAbstractGoal());
    // this.acceptedCallback.accept(goalHandle);
  }

  private static native RMWRequestId nativeTakeGoalRequest(
    long actionServerHandle,
    long requestFromJavaConverterHandle,
    long requestToJavaConverterHandle,
    long requestDestructorHandle,
    MessageDefinition requestMessage);

  private static native void nativeSendGoalResponse(
    long actionServerHandle,
    RMWRequestId header,
    long responseFromJavaConverterHandle,
    long responseToJavaConverterHandle,
    long responseDestructorHandle,
    MessageDefinition responseMessage);

  /**
   * {@inheritDoc}
   */
  public void execute() {
    if (this.isGoalRequestReady()) {
      Class<? extends GoalRequestDefinition> requestType = this.actionTypeInstance.getSendGoalRequestType();
      Class<? extends GoalResponseDefinition> responseType = this.actionTypeInstance.getSendGoalResponseType();

      GoalRequestDefinition requestMessage = null;
      GoalResponseDefinition responseMessage = null;

      try {
        requestMessage = requestType.newInstance();
        responseMessage = responseType.newInstance();
      } catch (InstantiationException ie) {
        ie.printStackTrace();
      } catch (IllegalAccessException iae) {
        iae.printStackTrace();
      }

      // GoalRequestDefinition requestMessage = new this.actionTypeInstance.SendGoalRequest();
      // GoalResponseDefinition responseMessage = new this.actionTypeInstance.SendGoalResponse();

      if (requestMessage != null && responseMessage != null) {
        long requestFromJavaConverterHandle = requestMessage.getFromJavaConverterInstance();
        long requestToJavaConverterHandle = requestMessage.getToJavaConverterInstance();
        long requestDestructorHandle = requestMessage.getDestructorInstance();
        long responseFromJavaConverterHandle = responseMessage.getFromJavaConverterInstance();
        long responseToJavaConverterHandle = responseMessage.getToJavaConverterInstance();
        long responseDestructorHandle = responseMessage.getDestructorInstance();

	// TODO: Use long instead?
        RMWRequestId rmwRequestId =
          nativeTakeGoalRequest(
            this.handle,
            requestFromJavaConverterHandle, requestToJavaConverterHandle, requestDestructorHandle,
            requestMessage);
        if (rmwRequestId != null) {
	  this.executeGoalCallback(rmwRequestId, requestMessage, responseMessage);
          nativeSendGoalResponse(
            this.handle, rmwRequestId,
            responseFromJavaConverterHandle, responseToJavaConverterHandle,
            responseDestructorHandle, responseMessage);
        }
      }
    }

    if (this.isCancelRequestReady()) {
      // TODO
    }

    if (this.isResultRequestReady()) {
      // TODO

    }

    if (this.isGoalExpiredReady()) {
      // TODO
    }
  }

  /**
   * Destroy the underlying rcl_action_server_t.
   *
   * @param nodeHandle A pointer to the underlying rcl_node_t handle that
   *     created this action server.
   * @param handle A pointer to the underlying rcl_action_server_t
   */
  private static native void nativeDispose(long nodeHandle, long handle);

  /**
   * {@inheritDoc}
   */
  public final void dispose() {
    Node node = this.nodeReference.get();
    if (node != null) {
      nativeDispose(node.getHandle(), this.handle);
      node.removeActionServer(this);
      this.handle = 0;
    }
  }

  /**
   * {@inheritDoc}
   */
  public final long getHandle() {
    return handle;
  }
}
