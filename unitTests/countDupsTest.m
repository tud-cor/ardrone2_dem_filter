% Main test function
function tests = countDupsTest
tests = functiontests(localfunctions);
end

% Local test functions
function testOneZero(testCase)
a = 0.0;
act = @()countDups(a);
exp = 'countDups:TooShortInput';
verifyError(testCase,act,exp);
end

function testTwoZeros(testCase)
a = [0.0,0.0];
act = countDups(a);
output.count = [1,0;2,1];
output.dups2 = 1;
exp = output;
verifyEqual(testCase,act,exp);
end

function testThreeOnes(testCase)
a = [1.0,1.0,1.0];
act = countDups(a);
output.count = [1,0;2,0;3,1];
output.dups3 = 1;
exp = output;
verifyEqual(testCase,act,exp);
end

function testTwiceSameNumber(testCase)
a = [1.0,2.0,1.0,1.0];
act = countDups(a);
output.count = [1,2;2,1];
output.dups1 = [1;2];
output.dups2 = 3;
exp = output;
verifyEqual(testCase,act,exp);
end

function testTwoSequences(testCase)
a = [1.0,1.0,1.0,1.0,1.0,2.0,2.0,2.0,2.0,2.0,2.0];
act = countDups(a);
output.count = [1,0;2,0;3,0;4,0;5,1;6,1];
output.dups5 = 1;
output.dups6 = 6;
exp = output;
verifyEqual(testCase,act,exp);
end

function testFirstArbitraryA(testCase)
a = [1.0,1.0,1.0,2.0,3.0,4.0,4.0,5.0];
act = countDups(a);
output.count = [1,3;2,1;3,1];
output.dups1 = [4;5;8];
output.dups2 = 6;
output.dups3 = 1;
exp = output;
verifyEqual(testCase,act,exp);
end

function testSecondArbitraryA(testCase)
a = [1.0,1.0,1.0,2.0,3.0,4.0,4.0,5.0,5.0];
act = countDups(a);
output.count = [1,2;2,2;3,1];
output.dups1 = [4;5];
output.dups2 = [6;8];
output.dups3 = 1;
exp = output;
verifyEqual(testCase,act,exp);
end

function testThreshold(testCase)
a = [0.0,1e-10,1e-11,1e-12,1e-11,0.0,1.0];
act = countDups(a);
output.count = [1,2;2,0;3,0;4,0;5,1];
output.dups1 = [1;7];
output.dups5 = 2;
exp = output;
verifyEqual(testCase,act,exp);
end

function testThreshold2(testCase)
a = [0.0,1e-11,1e-10,1e-12,1e-11,0.0,1.0];
act = countDups(a);
output.count = [1,1;2,0;3,0;4,0;5,0;6,1];
output.dups1 = 7;
output.dups6 = 1;
exp = output;
verifyEqual(testCase,act,exp);
end