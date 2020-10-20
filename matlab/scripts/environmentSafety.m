classdef environmentSafety
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
        peopleDetected;
        
    end
    properties(Access = private)
        people_sub
        people_msg;
    end
    

    
    methods
        
        function self = environmentSafety
        
            self.people_sub = rossubscriber('/person_detector/status','std_msgs/Bool');
           % self.people_msg = receive(self.people_pub.LatestMessage);
           %self.people_pub = rossubscriber('/person_detector/status','std_msgs/Bool');
           %self.peopleDetected = self.people_pub.LatestMessage.Data;
          % self.people_msg = receive(self.people_sub);
           %  self.peopleDetected =  self.people_msg.Data;
        end
%%        
        function status = getStatus(self)
            self.peopleDetected = self.people_sub.LatestMessage.Data
            status = self.peopleDetected;
          %self.people_msg = receive(self.people_sub);
          %self.peopleDetected =  self.people_msg.Data;
        end
        
        
    end
end

