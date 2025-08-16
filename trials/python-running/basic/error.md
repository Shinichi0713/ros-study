yoshishinnze@MyComputer:~/catkin_ws/src/ros_lecture$ source ~/catkin_ws/devel/setup.bash
yoshishinnze@MyComputer:~/catkin_ws/src/ros_lecture$ rosrun py_lecture python_talker.py _content:=data
Traceback (most recent call last):
  File "/home/yoshishinnze/catkin_ws/src/ros_lecture/py_lecture/scripts/python_talker.py", line 20, in `<module>`
    talker()
  File "/home/yoshishinnze/catkin_ws/src/ros_lecture/py_lecture/scripts/python_talker.py", line 7, in talker
    rospy.init_node('talker')
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/client.py", line 326, in init_node
    _init_node_params(argv, name)
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/client.py", line 186, in _init_node_params
    set_param(rosgraph.names.PRIV_NAME + param_name, param_value)
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/client.py", line 540, in set_param
    _param_server[param_name] = param_value #MasterProxy does all the magic for us
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/msproxy.py", line 136, in __setitem__
    self.target.setParam(rospy.names.get_caller_id(), resolved_key, val)
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1109, in __call__
    return self.__send(self.__name, args)
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/core.py", line 696, in _ServerProxy__request
    return xmlrpcclient.ServerProxy._ServerProxy__request(
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1450, in __request
    response = self.__transport.request(
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1153, in request
    return self.single_request(host, handler, request_body, verbose)
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1165, in single_request
    http_conn = self.send_request(host, handler, request_body, verbose)
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1278, in send_request
    self.send_content(connection, request_body)
  File "/usr/lib/python3.8/xmlrpc/client.py", line 1308, in send_content
    connection.endheaders(request_body)
  File "/usr/lib/python3.8/http/client.py", line 1251, in endheaders
    self._send_output(message_body, encode_chunked=encode_chunked)
  File "/usr/lib/python3.8/http/client.py", line 1011, in _send_output
    self.send(msg)
  File "/usr/lib/python3.8/http/client.py", line 951, in send
    self.connect()
  File "/usr/lib/python3.8/http/client.py", line 922, in connect
    self.sock = self._create_connection(
  File "/usr/lib/python3.8/socket.py", line 808, in create_connection
    raise err
  File "/usr/lib/python3.8/socket.py", line 796, in create_connection
    sock.connect(sa)
ConnectionRefusedError: [Errno 111] Connection refused
yoshishinnze@MyComputer:~/catkin_ws/src/ros_lecture$


解決：他プロセスが動作してなかっただけ
