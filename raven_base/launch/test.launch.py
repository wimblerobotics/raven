from launch import LaunchContext, LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import EnvironmentVariable, PythonExpression

def generate_launch_description():
  ld = LaunchDescription()
  v = EnvironmentVariable('foo').perform(LaunchContext())
  a = LogInfo(msg=f"foo is '{v}'")
  # a = LogInfo(msg=("foo is: ", EnvironmentVariable('foo')))
  ld.add_action(a)
  return ld