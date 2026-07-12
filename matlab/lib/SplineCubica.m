function [q, qd, qdd] = SplineCubica(tk, qk, tt)
% =========================================================================
%       Cubic Spline Interpolation (Book Formulas)
%
%       INPUT:
%           tk : vector of knot times      [1 x M]
%           qk : vector of knot positions  [1 x M]
%           tt : evaluation time vector    [1 x N]
%
%       OUTPUT:
%           q   : position trajectory      [1 x N]
%           qd  : velocity trajectory      [1 x N]
%           qdd : acceleration trajectory  [1 x N]
%
%       NOTE:
%       - Natural spline boundary conditions used:
%             q''(t0) = 0
%             q''(t_end) = 0
%       - Implements exactly the linear system from the book
% =========================================================================

M = length(qk);        % number of knots
h = diff(tk);          % interval lengths (M-1)
N = length(tt);        % number of evaluation points

% -------------------------------------------------------------
%   BUILD TRIDIAGONAL SYSTEM FOR z = q''(tk)
% -------------------------------------------------------------
A = zeros(M,M);
rhs = zeros(M,1);

% Boundary conditions (natural spline)
A(1,1)     = 1;
A(M,M)     = 1;
rhs(1)     = 0;
rhs(M)     = 0;

% Internal equations (from book)
for i = 2:M-1
    A(i,i-1) = h(i-1);
    A(i,i)   = 2 * (h(i-1) + h(i));
    A(i,i+1) = h(i);
    
    rhs(i)   = 6 * ( (qk(i+1)-qk(i))/h(i) - (qk(i)-qk(i-1))/h(i-1) );
end

% Solve for second derivatives
z = A \ rhs;

% -------------------------------------------------------------
%   EVALUATE SPLINE PIECEWISE ON TIME VECTOR tt
% -------------------------------------------------------------
q   = zeros(1,N);
qd  = zeros(1,N);
qdd = zeros(1,N);

for k = 1:M-1
    % indices where tt is in [tk(k), tk(k+1)]
    idx = find(tt >= tk(k) & tt <= tk(k+1));
    t   = tt(idx) - tk(k);

    hk  = h(k);
    
    % Book formulas
    q(idx) = ( z(k)/(6*hk) * (tk(k+1)-tt(idx)).^3 ) + ...
             ( z(k+1)/(6*hk) * (tt(idx)-tk(k)).^3 ) + ...
             ( (qk(k)/hk - z(k)*hk/6) * (tk(k+1)-tt(idx)) ) + ...
             ( (qk(k+1)/hk - z(k+1)*hk/6) * (tt(idx)-tk(k)) );

    % Velocity
    qd(idx) = (-z(k)/(2*hk) * (tk(k+1)-tt(idx)).^2 ) + ...
              ( z(k+1)/(2*hk) * (tt(idx)-tk(k)).^2 ) + ...
              ( (qk(k+1)-qk(k))/hk - (hk/6)*(z(k+1)-z(k)) );

    % Acceleration
    qdd(idx) = z(k) * (tk(k+1)-tt(idx))/hk + ...
               z(k+1) * (tt(idx)-tk(k))/hk;
end

end
