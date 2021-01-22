function runTests()
% RUNTESTS Run all unit tests for functions countDups, fixDups and
% interpolate and report how many tests failed. The tests are run a few
% times to avoid misleading results caused by random value generations
% inside the different test functions.
%
%   Author: Dennis Benders
%   Last edited: 08.05.2020
%
%   Input:  none
%
%   Output: text in Command Window indicating how many tests failed
%
%   Usage:  run "runTests()" in the Command Window,
%           or press F5 while the editor with this function is active


%------------------------------- Parameters -------------------------------
nTests = 1;
%--------------------------------------------------------------------------

%----------- Run tests for different functions and store results ----------
addpath('../');

for testNr = 1:nTests
    results(1,testNr).test = runtests('countDupsTest.m');
    results(2,testNr).test = runtests('fixDupsTest.m');
    results(3,testNr).test = runtests('interpolateTest.m');
end
%--------------------------------------------------------------------------

%---------------------- Count amount of failed tests ----------------------
nFailed = 0;
for i = 1:size(results,1)
    for j = 1:size(results,2)
        for k = 1:length(results(i,j).test)
            if results(i,j).test(k).Failed == 1
                nFailed = nFailed + 1;
            end
        end
    end
end
%--------------------------------------------------------------------------

%------------------------------ Print results -----------------------------
if nFailed == 0
    fprintf('\nAll tests passed!\n');
elseif nFailed == 1
    fprintf('\n1 test failed! %s\n', ...
            'Please take a look at the failure summary above.');
else
    fprintf('\n%d tests failed! %s\n', nFailed, ...
            'Please take a look at the failure summaries above.');
end
%--------------------------------------------------------------------------

end
