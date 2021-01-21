%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function to find start and end index
%
% Function to find the start and end array index of different recorded bag
% data signals in getExpData.m.
% 
% Author:        Dennis Benders, TU Delft, CoR
% Last modified: 21.01.2021
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [startIdx,endIdx] = findStartEndIdx(x,ref,floatTol)

startTime = ref(1);
endTime   = ref(end);

for i = 1:length(x)-1
%     if i == 1 && x(1) > startTime
%         startIdx = 1;
%         break;
%     elseif abs(x(i)-startTime) < floatTol
%         startIdx = i;
%         break;
    if x(i) < startTime && x(i+1) > startTime
        startIdx = i;
        break;
    end
end

for i = 2:length(x)
%     if abs(x(i)-endTime) < floatTol
%         endIdx = i;
%         break;
    if x(i-1) < endTime && x(i) > endTime
        endIdx = i;
        break;
%     elseif i == length(ref) && x(i) < endTime
%         endIdx = i;
%         break;
    end
end

end
