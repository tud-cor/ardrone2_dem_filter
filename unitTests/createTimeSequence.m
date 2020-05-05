function output = createTimeSequence(sampleFreq, dur, offset, putDups)
if nargin < 4
    putDups = 0;
end

sampleTime = 1/sampleFreq;

n = sampleFreq*dur + 1;

output = zeros(1,n);
output(1) = offset;

if putDups
    randSeq = round(rand(n,1));
end

for i = 2:n
    if putDups && randSeq(i)
        output(i) = output(i-1);
    else
        output(i) = output(1) + (i-1)*sampleTime;
    end
end
end