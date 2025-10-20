close all
load("Results_Driving_Test_250928_1.mat")
ResultsDrivingTests_1 = ResultsDrivingTests;
load("Results_Driving_Test_250928_3.mat")
ResultsDrivingTests_3 = ResultsDrivingTests;


figure('Position', [100, 100, 1000, 600]);
subplot(2,1,1)
plot(ResultsDrivingTests_1.AccelPedalCmd.Time,ResultsDrivingTests_1.AccelPedalCmd.Data,'r')
hold on
plot(ResultsDrivingTests_3.AccelPedalCmd.Time,ResultsDrivingTests_3.AccelPedalCmd.Data,'-.b')
legend('Model1','Model3','fontsize',12, 'Location', 'best', 'FontName', 'Times new roman')
set(gca,'fontsize',15,'linewidth',1, 'FontName', 'Times new roman')
ylabel('AccPedal / [%]','fontsize',15, 'FontName', 'Times new roman')
xlabel('time / [s]','fontsize',15, 'FontName', 'Times new roman')
set(gca,'GridLineStyle','--','GridColor','k','GridAlpha',1);
set(gca,'xticklabel',get(gca,'xtick'));

subplot(2,1,2)
plot(ResultsDrivingTests_1.BrakePedalCmd.Time,ResultsDrivingTests_1.BrakePedalCmd.Data,'r')
hold on
plot(ResultsDrivingTests_3.BrakePedalCmd.Time,ResultsDrivingTests_3.BrakePedalCmd.Data,'-.b')
legend('Model1','Model3','fontsize',12, 'Location', 'best', 'FontName', 'Times new roman')
set(gca,'fontsize',15,'linewidth',1, 'FontName', 'Times new roman')
ylabel('BrakePedal / [%]','fontsize',15, 'FontName', 'Times new roman')
xlabel('time / [s]','fontsize',15, 'FontName', 'Times new roman')
set(gca,'GridLineStyle','--','GridColor','k','GridAlpha',1);
set(gca,'xticklabel',get(gca,'xtick'));



