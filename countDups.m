function output = countDups(input)
% Check input
if length(input) <= 1
    error('countDups:TooShortInput', 'Input should have at least %s.', ...
        'a length of 2.');
end

% Initialise output
output.count = zeros(1,2);
output.count(1,1) = 1;
l = size(output.count,1);

% Iterate through all input array elements to detect duplicates (sequences 
% of the same numbers)
n = length(input);
i = 2;
while i <= n
    j = 1;
    
    % Set i to index after a sequence of the same numbers in an array
    while abs(input(i) - input(i-1)) < 1e-10
        j = j + 1;
        if i == n
            i = i + 1;
            break
        end
        i = i + 1;
    end
    
    % If a sequence of more numbers than before is detected, expand
    % output.count
    if j > l
        tempL = l;
        tempR = output.count;
        output.count = zeros(j,2);
        l = j;
        output.count(1:tempL,:) = tempR;
        for k = tempL:j
            output.count(k,1) = k;
        end
        clear k tempL tempR;
    end
    output.count(j,2) = output.count(j,2) + 1;
    
    % Create a struct for the beginning positions of the sequences detected
    % above
    tempString = ['dups' num2str(j)];
    if isfield(output, tempString)
        output.(tempString) = [output.(tempString); i-j];
    else
        output.(tempString) = i-j;
    end
    
    % If the last number is not a sequence, but a single occurring number,
    % take this one also into account (while loop ends hereafter)
    if i == n && abs(input(i) - input(i-1)) > 1e-10
        output.count(1,2) = output.count(1,2) + 1;
        if isfield(output, 'dups1')
            output.dups1 = [output.dups1; n];
        else
            output.dups1 = n;
        end
    end
    
    i = i + 1;
end

output = orderfields(output);

% Check result, otherwise display warning
totElCnt = 0;
for i = 1:l
    totElCnt = totElCnt + output.count(i,1)*output.count(i,2);
end
if totElCnt ~= n
    error('countDups:UnmatchedResult', 'Amount of elements in %s.', ...
        'result does not equal amount of elements in the input array');
end
end