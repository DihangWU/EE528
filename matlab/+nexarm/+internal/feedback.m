function [measured, previous, reference] = feedback(measured, previous, reference, n, horizon)
%FEEDBACK Check finite feedback, issued command, and reference preview.
validateattributes(measured, {'numeric'}, {'real','finite','vector','numel',n});
validateattributes(previous, {'numeric'}, {'real','finite','vector','numel',n});
validateattributes(reference, {'numeric'}, {'real','finite','size',[horizon n]});
measured = double(measured(:).'); previous = double(previous(:).');
reference = double(reference);
end
