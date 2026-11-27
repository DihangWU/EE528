function results=run_tests()
%RUN_TESTS Execute the MATLAB behavioral and cross-language regression suite.
root=fileparts(mfilename('fullpath')); addpath(root); setup();
results=runtests(fullfile(root,'tests')); disp(table(results));
assert(all([results.Passed]),'nexarm:TestsFailed','One or more MATLAB tests failed.');
end
