% Area above the curve is reachable from current state without circling
% Conditions:
%   - no wind
% Limitations:
%   - does not account for the dynamics of rolling motion
%
% (Directly comes from the turn radius limit)

function xt_x = xt_x_f(angle, radius)
    xt_x = transpose(1-cos(angle*pi/180))*radius;
endfunction

function xt_y = xt_y_f(angle, radius)
    xt_y = transpose(sin(angle*pi/180))*radius;
endfunction


alpha = 0:1:90;
radius = 70;

xt_x_rel = xt_x_f(alpha, radius);
xt_y_rel = xt_y_f(alpha, radius);

plot(xt_x_rel, xt_y_rel,"-",-xt_x_rel,xt_y_rel,"-");

xt_x_f(alpha,[60,80,100])(5:10,:) % show results for all radii and angles between 5 and 10

% TODO Non-linear pitch control (e.g. rapid compensation of above glidepath condition)


% TODO Calculate the area reachable from current position, knowing the roll motion dynamics (even simplified)
