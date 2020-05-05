function output = simpleInterpolate(inputArray, refDataRatio)    
l = length(inputArray);
output = zeros(1,(l-1)*refDataRatio+1);
for i = 1:l-1
    for j = 1:refDataRatio
        output(refDataRatio*(i-1)+j) = inputArray(i) + ...
            (j-1)/refDataRatio*(inputArray(i+1) - inputArray(i));
    end
end
output(end) = inputArray(end);
end