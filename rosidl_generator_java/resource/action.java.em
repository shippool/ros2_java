@# Generation triggered from rosidl_generator_java/resource/idl.java.em
// generated from rosidl_generator_java/resource/action.java.em
// with input from @(package_name):@(interface_path)
// generated code does not contain a copyright notice

package @(package_name + '.' + interface_path.parts[0]);
@{
import os
from rosidl_cmake import expand_template

namespaces = action.namespaced_type.namespaces
type_name = action.namespaced_type.name
goal_type_name = action.goal.structure.namespaced_type.name
result_type_name = action.result.structure.namespaced_type.name
feedback_type_name = action.feedback.structure.namespaced_type.name
send_goal_type_name = action.send_goal_service.namespaced_type.name
get_result_type_name = action.get_result_service.namespaced_type.name

data = {
    'package_name': package_name,
    'interface_path': interface_path,
    'output_dir': output_dir,
    'template_basepath': template_basepath,
}
data.update({'message': action.goal})
output_file = os.path.join(output_dir, *namespaces[1:], goal_type_name + '.java')
expand_template(
    'msg.java.em',
    data,
    output_file,
    template_basepath=template_basepath)

data.update({'message': action.result})
output_file = os.path.join(output_dir, *namespaces[1:], result_type_name + '.java')
expand_template(
    'msg.java.em',
    data,
    output_file,
    template_basepath=template_basepath)

data.update({'message': action.feedback})
output_file = os.path.join(output_dir, *namespaces[1:], feedback_type_name + '.java')
expand_template(
    'msg.java.em',
    data,
    output_file,
    template_basepath=template_basepath)

data.update({'service': action.send_goal_service})
output_file = os.path.join(output_dir, *namespaces[1:], send_goal_type_name + '.java')
expand_template(
    'srv.java.em',
    data,
    output_file,
    template_basepath=template_basepath)

data.update({'service': action.get_result_service})
output_file = os.path.join(output_dir, *namespaces[1:], get_result_type_name + '.java')
expand_template(
    'srv.java.em',
    data,
    output_file,
    template_basepath=template_basepath)

action_imports = [
    'java.util.List',
    'org.ros2.rcljava.common.JNIUtils',
    'org.ros2.rcljava.interfaces.ActionDefinition',
    'org.ros2.rcljava.interfaces.GoalRequestDefinition',
    'org.ros2.rcljava.interfaces.GoalResponseDefinition',
    'org.ros2.rcljava.interfaces.MessageDefinition',
    'org.slf4j.Logger',
    'org.slf4j.LoggerFactory',
]
}@

@[for action_import in action_imports]@
import @(action_import);
@[end for]@

public class @(type_name) implements ActionDefinition {

  public static class Goal extends @(type_name)_Goal implements ActionGoal<@(type_name)> {}

  public static class Result extends @(type_name)_Result implements ActionResult<@(type_name)> {}

  public static class Feedback extends @(type_name)_Feedback implements ActionFeedback<@(type_name)> {}

  public static class SendGoalRequest extends @(type_name)_SendGoal_Request implements GoalRequestDefinition {
    public List<Byte> getGoalUuid() {
      return super.getGoalId().getUuid();
    }
  }

  public static class SendGoalResponse extends @(type_name)_SendGoal_Response implements GoalResponseDefinition {
    public void accept(boolean accepted) {
      super.setAccepted(accepted);
    }
  }

  public Class<? extends GoalRequestDefinition> getSendGoalRequestType() {
    return SendGoalRequest.class;
  }

  public Class<? extends GoalResponseDefinition> getSendGoalResponseType() {
    return SendGoalResponse.class;
  }

  public Class<? extends MessageDefinition> getGetResultRequestType() {
    return @(type_name)_GetResult_Request.class;
  }

  public Class<? extends MessageDefinition> getGetResultResponseType() {
    return @(type_name)_GetResult_Response.class;
  }

  private static final Logger logger = LoggerFactory.getLogger(@(type_name).class);

  static {
    try {
      JNIUtils.loadTypesupport(@(type_name).class);
    } catch (UnsatisfiedLinkError ule) {
      logger.error("Native code library failed to load.\n" + ule);
      System.exit(1);
    }
  }

  public static native long getActionTypeSupport();
}
