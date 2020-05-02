function runTests()
results(1).test = runtests('countDupsTest.m');
results(2).test = runtests('fixDupsTest.m');

failedFlag = 0;
for i = 1:length(results)
    for j = 1:length(results(i).test)
        if results(i).test(j).Failed == 1
            failedFlag = 1;
        end
    end
end
if ~failedFlag
    fprintf('\nAll tests passed!\n');
else
    fprintf('\nNot all tests passed! %s', ...
            'Please take a look at the failure summary.\n');
end
end