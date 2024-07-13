
function mask = getMaskMultiple(PCs_Ts, K)
%assert (length(PCs)== length(Ts),"Length of pointclouds not equal to transformations");
H = 260; W =346;
num_obj = size(PCs_Ts, 1);
mask = zeros(H,W);
depth = 10000*ones(H, W);
for j=1:num_obj
    PC = PCs_Ts{j,1}; T = PCs_Ts{j,2};
    U = K*T*PC';
    X = [U(1,:)./U(3,:); U(2,:)./U(3,:)];
    X = floor(X);
    for i = 1:size(X,2)
        if X(2,i) < 1 || X(2,i) > H || X(1,i)<1 || X(1,i) > W
    %         display("error");
            continue;
        end
        if U(3,i) < depth(X(2,i), X(1,i))
            mask(X(2,i), X(1,i)) = j;
            depth(X(2,i), X(1,i)) = U(3,i);
        end
    end
end
end