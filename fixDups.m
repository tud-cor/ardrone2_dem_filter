function output = fixDups(input)
% FIXDUPS Resolve the duplicated numbers in the input array by assuming
% linearly increasing numbers (because input is often a time array). The
% input array should contain at least 4 unique numbers to have a proper
% calculation of the increment size.
%
%   Author: Dennis Benders
%   Last edited: 08.05.2020
%
%   Input:  input   array containing duplications, which need to be
%                   resolved
%
%   Output: output  array with resolved duplications (only unique numbers)
%
%   For usage, see interpolate.m.


%---------------------------- Initialise output ---------------------------
output = input;
%--------------------------------------------------------------------------


%------------------ Get average distance between samples ------------------
dupsInfo = countDups(input);
if isfield(dupsInfo, 'dups1')
    nDups1 = length(dupsInfo.dups1);
    if nDups1 < 4
        error('fixDups:TooFewUnique', ...
              'The input contains less than 4 unique numbers. %s', ...
              'Unable to perform fix.');
    end
    avgDelta = (input(dupsInfo.dups1(ceil(nDups1/2))) - input(1))/...
               (dupsInfo.dups1(ceil(nDups1/2)) - 1);
else
    error('fixDups:NoUnique', ...
          'The input contains no unique numbers. Unable to perform fix.');
end
%--------------------------------------------------------------------------


% -------------------------- Resolve duplications -------------------------
% Add the average distance to the duplicated samples with respect to the
% start of their sequence in order to equalise the data

nDupsTot = size(dupsInfo.count,1);

% For each amount of duplicates in sequence (starting from 2, of course)
for i = 2:nDupsTot
    
    % For each duplicate sequnce of a certain length
    nDups = dupsInfo.count(i,2);
    if nDups ~= 0
        tempString = ['dups' num2str(i)];
        for j = 1:nDups
            
            % For each element in the duplicate sequence
            for k = 1:i-1
                output(dupsInfo.(tempString)(j)+k) = ...
                    output(dupsInfo.(tempString)(j)) + k*avgDelta;
            end  
        end
    end
end
%--------------------------------------------------------------------------
end