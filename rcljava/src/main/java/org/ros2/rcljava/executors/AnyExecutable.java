/* Copyright 2017-2018 Esteve Fernandez <esteve@apache.org>
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

package org.ros2.rcljava.executors;

import org.ros2.rcljava.action.ActionServer;
import org.ros2.rcljava.client.Client;
import org.ros2.rcljava.subscription.Subscription;
import org.ros2.rcljava.service.Service;
import org.ros2.rcljava.timer.Timer;

public class AnyExecutable {
  public Timer timer;
  public Subscription subscription;
  public Service service;
  public Client client;
  public ActionServer actionServer;
}
