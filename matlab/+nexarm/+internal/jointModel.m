function [lower, upper, delta] = jointModel(lower, upper, velocity, dt)
%JOINTMODEL Validate and normalize the common controller interface.
validateattributes(dt, {'numeric'}, {'real','finite','scalar','positive'});
validateattributes(lower, {'numeric'}, {'real','finite','vector','nonempty'});
validateattributes(upper, {'numeric'}, {'real','finite','vector','numel',numel(lower)});
validateattributes(velocity, {'numeric'}, {'real','finite','positive','vector','numel',numel(lower)});
lower = double(lower(:).'); upper = double(upper(:).');
assert(all(lower < upper), 'nexarm:InvalidBounds', 'Every lower bound must be below its upper bound.');
delta = double(velocity(:).') * dt;
end
