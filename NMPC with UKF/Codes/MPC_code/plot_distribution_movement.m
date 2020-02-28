function plot_distribution_movement(means,covar, iterMax)
    
    N = 3;
    x1 = -N:0.1:N;
    x2 = -N:0.1:N;
    [X1,X2] = meshgrid(x1,x2);
    X = [X1(:) X2(:)];

    for i = 2:iterMax

        mu = means{i};
        sigma = covar{i};

        y = mvnpdf(X,mu,sigma);
        y = reshape(y,length(x2),length(x1));

        surf(x1,x2,y)
        caxis([min(y(:))-0.5*range(y(:)),max(y(:))])
        axis([-2*N 2*N -2*N 2*N 0 0.4])
        xlabel('x1')
        ylabel('x2')
        zlabel('Probability Density')

        pause(0.001);

    end

% for i = 1:N
%     X = mvnrnd([0 0]+i,eye(2), 1000);
%     histogram2(X(:,1),X(:,2),'Normalization','pdf');
%     pause(0.001);
% end
end