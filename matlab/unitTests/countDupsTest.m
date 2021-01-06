%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% countDups unit test file
%
% This file contains test functions according to the MATLAB unit test
% framework to properly check the functionality of function countDups.
%
% Author: Dennis Benders
% Last edited: 08.05.2020
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main test function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function tests = countDupsTest
tests = functiontests(localfunctions);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End main test function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Local test functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------- Errors ---------------------------------
function testOneZero(testCase)
a = 0.0;
act = @()countDups(a);
exp = 'countDups:TooShortInput';
verifyError(testCase,act,exp);
end
%--------------------------------------------------------------------------

%---------------------------- Only duplications ---------------------------
function testTwoZeros(testCase)
a = [0.0,0.0];
act = countDups(a);
exp.count = [1,0;2,1];
exp.dups2 = 1;
verifyEqual(testCase,act,exp);
end

function testThreeOnes(testCase)
a = [1.0,1.0,1.0];
act = countDups(a);
exp.count = [1,0;2,0;3,1];
exp.dups3 = 1;
verifyEqual(testCase,act,exp);
end

function testTwoSequences(testCase)
a = [1.0,1.0,1.0,1.0,1.0,2.0,2.0,2.0,2.0,2.0,2.0];
act = countDups(a);
exp.count = [1,0;2,0;3,0;4,0;5,1;6,1];
exp.dups5 = 1;
exp.dups6 = 6;
verifyEqual(testCase,act,exp);
end
%--------------------------------------------------------------------------

%------------------------ Same number occurs twice ------------------------
function testTwiceSameNumber(testCase)
a = [1.0,2.0,1.0,1.0];
act = countDups(a);
exp.count = [1,2;2,1];
exp.dups1 = [1;2];
exp.dups2 = 3;
verifyEqual(testCase,act,exp);
end
%--------------------------------------------------------------------------

%--------------------------- Arbitrary sequences --------------------------
function testFirstArbitraryA(testCase)
a = [1.0,1.0,1.0,2.0,3.0,4.0,4.0,5.0];
act = countDups(a);
exp.count = [1,3;2,1;3,1];
exp.dups1 = [4;5;8];
exp.dups2 = 6;
exp.dups3 = 1;
verifyEqual(testCase,act,exp);
end

function testSecondArbitraryA(testCase)
a = [1.0,1.0,1.0,2.0,3.0,4.0,4.0,5.0,5.0];
act = countDups(a);
exp.count = [1,2;2,2;3,1];
exp.dups1 = [4;5];
exp.dups2 = [6;8];
exp.dups3 = 1;
verifyEqual(testCase,act,exp);
end
%--------------------------------------------------------------------------

%------------------------- Floating-point accuracy ------------------------
function testThreshold(testCase)
a = [0.0,1e-10,1e-11,1e-12,1e-11,0.0,1.0];
act = countDups(a);
exp.count = [1,2;2,0;3,0;4,0;5,1];
exp.dups1 = [1;7];
exp.dups5 = 2;
verifyEqual(testCase,act,exp);
end

function testThreshold2(testCase)
a = [0.0,1e-11,1e-10,1e-12,1e-11,0.0,1.0];
act = countDups(a);
exp.count = [1,1;2,0;3,0;4,0;5,0;6,1];
exp.dups1 = 7;
exp.dups6 = 1;
verifyEqual(testCase,act,exp);
end
%--------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End local test functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%