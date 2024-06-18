% 예제 행렬 B
B = [1 2; 3 4];

% 파라미터
N = 3; % N에 해당하는 행렬을 만들 예정

% 초기화
A = eye(size(B)); % 단위 행렬

% 행렬 생성
resultMatrix = [];
for k = 0:N-1
    % A^k * B 행렬 생성
    currentBlock = A^k * B;
    
    % 0으로 채워진 빈 행렬 생성
    zerosBlock = zeros(size(B, 1), size(B, 2) * k);
    
    % 결과 행렬에 추가
    resultMatrix = blkdiag(resultMatrix, [currentBlock, zerosBlock]);
end
