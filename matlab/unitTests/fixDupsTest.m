%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% fixDups unit test file
%
% This file contains test functions according to the MATLAB unit test
% framework to properly check the functionality of function fixDups.
%
% Author: Dennis Benders
% Last edited: 08.05.2020
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Main test function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function tests = fixDupsTest
tests = functiontests(localfunctions);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End main test function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Local test functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------- Errors ---------------------------------
function testTooFewUnique(testCase)
a = [0.0,1.0,1.0,3.0];
act = @()fixDups(a);
exp = 'fixDups:TooFewUnique';
verifyError(testCase,act,exp);
end

function testNoUnique(testCase)
a = [0.0,0.0,2.0,2.0];
act = @()fixDups(a);
exp = 'fixDups:NoUnique';
verifyError(testCase,act,exp);
end
%--------------------------------------------------------------------------

%--------------------------- Simple duplications --------------------------
function testSimpleUpcount(testCase)
a = [0.0,1.0,1.0,3.0,4.0,5.0];
act = fixDups(a);
exp = [0.0,1.0,2.0,3.0,4.0,5.0];
verifyEqual(testCase,act,exp);
end

function testSimpleUpcount2(testCase)
a = [0.0,1.0,1.0,3.0,3.0,5.0,6.0,7.0];
act = fixDups(a);
exp = [0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0];
verifyEqual(testCase,act,exp);
end

function testkHz(testCase)
a = [0.0,0.0,0.0,0.0,0.004,0.005,0.006,0.007,0.007,0.009,0.009,0.011];
act = fixDups(a);
exp = [0.0,0.001,0.002,0.003,0.004,0.005,0.006,0.007,0.008,0.009,0.010, ...
       0.011];
verifyEqual(testCase,act,exp,'AbsTol',1e-11);
end
%--------------------------------------------------------------------------

%------------------------- Floating-point accuracy ------------------------
function testThreshold(testCase)
a = [0.0,1e-12,1e-11,1e-12,1e-11,0.005,0.006,0.007,0.008];
act = fixDups(a);
exp = [0.0,0.001,0.002,0.003,0.004,0.005,0.006,0.007,0.008];
verifyEqual(testCase,act,exp);
end

function testThreshold2(testCase)
a = [0.0,0.001,0.001+1e-13,0.001+1e-12,0.001+1e-11,0.005,0.006,0.007, ...
     0.008];
act = fixDups(a);
exp = [0.0,0.001,0.002,0.003,0.004,0.005,0.006,0.007,0.008];
verifyEqual(testCase,act,exp);
end
%--------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End local test functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
