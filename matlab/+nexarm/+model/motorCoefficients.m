function [tau, a] = motorCoefficients(tau, n, dt)
%MOTORCOEFFICIENTS Exact zero-order-hold first-order motor response.
validateattributes(tau, {'numeric'}, {'real','finite','nonnegative','vector'});
if isscalar(tau), tau = repmat(tau, 1, n); end
assert(numel(tau) == n, 'nexarm:InvalidTau', 'Supply a scalar or one tau per servo.');
validateattributes(dt, {'numeric'}, {'real','finite','scalar','positive'});
tau = double(tau(:).'); a = zeros(1,n);
positive = tau > 0; a(positive) = exp(-dt ./ tau(positive));
end
