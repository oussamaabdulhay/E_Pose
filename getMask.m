function mask = getMask(PC, T, K)
H = 260; W =346; 
mask = zeros(H,W);
U = K*T*PC';
X = [U(1,:)./U(3,:); U(2,:)./U(3,:)];
X = floor(X);
for i = 1:size(X,2)
    if X(2,i) < 1 || X(2,i) > H || X(1,i)<1 || X(1,i) > W
%         display("error");
        continue;
    end
    mask(X(2,i), X(1,i)) = 1;
end
end