rosinit
%%

sub1 = rossubscriber('/person_detector/debug');
sub2 = rossubscriber('/person_detector/status');

%%

folderpath = '/home/esteban/git/fetchRobotRecycle/People_tracking_depencies';
%%
rosgenmsg(folderpath);
