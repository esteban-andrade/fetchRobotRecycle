rosinit
%%

%sub1 = rossubscriber('/person_detector/debug');
sub2 = rossubscriber('/person_detector/status','std_msgs/Bool');

%%
sub2.LatestMessage.Data

%%
msg = receive(sub2)
status= msg.Data
%%
folderpath = '/home/esteban/git/fetchRobotRecycle/People_tracking_depencies';
%%
rosgenmsg(folderpath);
