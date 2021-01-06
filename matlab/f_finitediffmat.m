%% Finite difference matrix
%{

Iris Hijne
August 2020

INFO______________________________________________________________________
This function generates a finite difference matrix that approximates the
derivatives of a signal when multiplied by an array of surrounding values,
using Taylor Expansion with a forward, central or backward difference.

INPUTS____________________________________________________________________
dt      [1xn]: the sampling time [s] (>0)
p       [1x1]: the embedding order (int>=1) (number of derivatives)
o       [1x1]: the error order (int>=1)
n       [1x1]: the signal dimension (int>=1)
method [str]: the method: 'f' forward, 'c' central, 'b' backward

OUTPUTS___________________________________________________________________
E [n(p+1) x n(p+o(+1))]: The finite difference matrix

%}

%%
function E = f_finitediffmat(dt,p,o,n,method)

pp = p+1; % number of rows in the matrix E for a 1­dim signal
switch method
% s: # samples required for the approx. of all desired derivatives
% E1: preallocation of matrix E for a 1­dim signal
% E1(,)=1: prepare first row of E1 to pass y onto itself in y_
    case 'f'
        s = p+o;
        if p==0; E1 = 1;
        else; E1 = zeros(pp,s); E1(1,1) = 1;
        end
    case 'c'
        if mod(p+o,2) == 0 % when p+o is even, o is increased by 1
            s = p+o+1;
        else
            s = p+o;
        end
        if p==0; E1 = 1;
        else; E1 = zeros(pp,s); E1(1,ceil(s/2)) = 1;
        end
    case 'b'
        s = p+o;
        if p==0; E1 = 1;
        else; E1 = zeros(pp,s); E1(1,end) = 1;
        end
end
C = zeros(1,s); % preallocation of array w/ coef for finite differences

for d = 1:p % we visit all p­values so we have all the derivatives
    switch method
    % sd: # samples required for the current derivative
    % jmin, jmax: required finite differences around the point of interest
    % imax: total number of samples (­1) required for the approximation
    % sumij: preallocation of the matrix for computation of C
    % sumijC: array with outcomes of the sums in sumij
    case 'f'
        sd = d+o;
        jmin = 0; jmax = sd-1;
        imax = sd-1;
        sumij = zeros(sd,sd);
        sumijC = zeros(size(sumij,2),1);
    case 'c'
        if mod(d+o,2) == 0 % when d+o is even, o is increased by 1
            sd = d+o+1;
        else
            sd = d+o;
        end
        jmax = (sd-1)/2; jmin = -jmax;
        imax = sd-1;
        sumij = zeros(sd,sd);
        sumijC = zeros(size(sumij,2),1);
    case 'b'
        sd = d+o;
        jmin = -(sd-1); jmax = 0;
        imax = sd-1;
        sumij = zeros(sd,sd);
        sumijC = zeros(size(sumij,2),1);
    end
    sumijC(d+1) = 1; % the sum must be 1 for j = d, 0 otherwise
    jrange = jmin:jmax; % range of all the finite difference elements
    for i = 0:imax % filling the matrix w/ elements to sum
        sumij(i+1,:) = jrange.^i;
    end
    switch method % computing C (solving the linear system)
        case 'f'
            C(1:sd) = (sumij\sumijC)';
        case 'c'
            C((s-sd)/2+1:s-(s-sd)/2) = (sumij\sumijC)';
        case 'b'
            C(s-sd+1:end) = (sumij\sumijC)';
    end
    E1(d+1,:) = (factorial(d)/dt^d).*C; % adding the elements to E
end

E = kron(E1,eye(n)); % E for an n­dim signal
end