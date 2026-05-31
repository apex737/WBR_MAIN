% B 행렬 (입력 자코비안) 추출 
function fu = calculate_fu(M, B)
    fu = zeros(4,2);
    fu(2:4,:) = M \ B;
end