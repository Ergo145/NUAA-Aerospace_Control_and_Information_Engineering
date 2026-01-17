classdef CRTBP_App_Final < matlab.apps.AppBase

    % 属性定义：UI组件
    properties (Access = public)
        UIFigure             matlab.ui.Figure
        GridLayout           matlab.ui.container.GridLayout
        LeftPanel            matlab.ui.container.Panel
        RightTabs            matlab.ui.container.TabGroup
        TabOrbit             matlab.ui.container.Tab
        TabEnergy            matlab.ui.container.Tab
        AxOrbit              matlab.ui.control.UIAxes
        AxEnergy             matlab.ui.control.UIAxes
        
        % 左侧布局容器
        MainScrollLayout     matlab.ui.container.GridLayout
        
        % 输入控件
        LabelMu              matlab.ui.control.Label
        EditMu               matlab.ui.control.NumericEditField
        LabelX0              matlab.ui.control.Label
        EditX0               matlab.ui.control.NumericEditField
        LabelY0              matlab.ui.control.Label
        EditY0               matlab.ui.control.NumericEditField
        LabelVx0             matlab.ui.control.Label
        EditVx0              matlab.ui.control.NumericEditField
        LabelVy0             matlab.ui.control.Label
        EditVy0              matlab.ui.control.NumericEditField
        LabelTMax            matlab.ui.control.Label
        EditTMax             matlab.ui.control.NumericEditField
        
        % 雅可比能量控制
        PanelC               matlab.ui.container.Panel
        CheckBoxAutoV        matlab.ui.control.CheckBox
        EditTargetC          matlab.ui.control.NumericEditField
        LabelRealC           matlab.ui.control.Label
        
        % 按钮
        BtnSimulate          matlab.ui.control.Button
        BtnPlotEnergy        matlab.ui.control.Button
        
        % 批量实验
        PanelExp             matlab.ui.container.Panel
        BtnExp1              matlab.ui.control.Button 
        BtnExp2              matlab.ui.control.Button 
        
        % 分隔符
        LabelSep             matlab.ui.control.Label
    end

    methods (Access = private)

        % --- 核心物理计算函数 ---
        function val = calc_omega(~, x, y, mu)
            r1 = sqrt((x + mu).^2 + y.^2);
            r2 = sqrt((x - 1 + mu).^2 + y.^2);
            val = 0.5 * (x.^2 + y.^2) + (1 - mu)./r1 + mu./r2;
        end

        function dYdt = equations_of_motion(~, t, Y, mu)
            x = Y(1); y = Y(2); vx = Y(3); vy = Y(4);
            r1 = sqrt((x + mu)^2 + y^2);
            r2 = sqrt((x - 1 + mu)^2 + y^2);
            ax = 2*vy + x - (1-mu)*(x+mu)/r1^3 - mu*(x-1+mu)/r2^3;
            ay = -2*vx + y - (1-mu)*y/r1^3 - mu*y/r2^3;
            dYdt = [vx; vy; ax; ay];
        end

        function [value, isterminal, direction] = detectCrash(~, t, Y, mu)
            x = Y(1); y = Y(2);
            r1 = sqrt((x + mu)^2 + y^2);
            r2 = sqrt((x - 1 + mu)^2 + y^2);
            value = [r1 - 1e-3; r2 - 1e-3];
            isterminal = [0; 0]; % 不停止积分，继续绘制
            direction = [0; 0];
        end

        function [vy0, success] = calculate_vy_from_c(app, x, y, C, mu)
            Omega = app.calc_omega(x, y, mu);
            v_sq = 2 * Omega - C;
            if v_sq < 0
                vy0 = 0;
                success = false;
            else
                vy0 = sqrt(v_sq);
                success = true;
            end
        end

        % --- 绘图辅助 ---
        function plot_background_on_axes(app, target_axes, mu, C)
            [xx, yy] = meshgrid(linspace(-1.6, 1.6, 200), linspace(-1.6, 1.6, 200));
            zz = app.calc_omega(xx, yy, mu);
            zz_limit = 2 * zz; 
            
            contour(target_axes, xx, yy, zz_limit, [C C], 'k', 'LineWidth', 1.5);
            hold(target_axes, 'on');
            contourf(target_axes, xx, yy, zz_limit, [0 C], 'FaceColor', [0.8 0.8 0.8], 'FaceAlpha', 0.5, 'LineStyle', 'none');
            
            plot(target_axes, -mu, 0, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
            plot(target_axes, 1-mu, 0, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
        end

        % --- 回调函数 ---
        function CheckBoxAutoVValueChanged(app, src, event)
            if app.CheckBoxAutoV.Value
                app.EditVy0.Enable = 'off';
                app.EditTargetC.Enable = 'on';
                app.EditVx0.Value = 0; 
                app.EditVx0.Enable = 'off';
            else
                app.EditVy0.Enable = 'on';
                app.EditTargetC.Enable = 'off';
                app.EditVx0.Enable = 'on';
            end
        end

        function PlotEnergyButtonPushed(app, src, event)
            mu = app.EditMu.Value;
            [x, y] = meshgrid(linspace(-2, 2, 80), linspace(-2, 2, 80));
            z = app.calc_omega(x, y, mu);
            z(z > 4) = 4; 
            
            % 清空并绘图
            cla(app.AxEnergy);
            surf(app.AxEnergy, x, y, z, 'EdgeColor', 'none');
            colormap(app.AxEnergy, 'jet');
            colorbar(app.AxEnergy);
            title(app.AxEnergy, ['等效势能 (\mu = ' num2str(mu) ')']);
            xlabel(app.AxEnergy, 'x'); ylabel(app.AxEnergy, 'y'); zlabel(app.AxEnergy, '\Omega');
            view(app.AxEnergy, -30, 60);
            
            % 切换Tab
            app.RightTabs.SelectedTab = app.TabEnergy;
        end

        function SimulateButtonPushed(app, src, event)
            mu = app.EditMu.Value;
            x0 = app.EditX0.Value;
            y0 = app.EditY0.Value;
            t_max = app.EditTMax.Value;
            
            if app.CheckBoxAutoV.Value
                targetC = app.EditTargetC.Value;
                [vy0, success] = app.calculate_vy_from_c(x0, y0, targetC, mu);
                if ~success
                    uialert(app.UIFigure, '给定能量不足 (v^2 < 0)', '错误');
                    return;
                end
                vx0 = 0;
                app.EditVy0.Value = vy0; 
                C = targetC;
            else
                vx0 = app.EditVx0.Value;
                vy0 = app.EditVy0.Value;
                omega = app.calc_omega(x0, y0, mu);
                v_sq = vx0^2 + vy0^2;
                C = 2*omega - v_sq;
            end
            
            app.LabelRealC.Text = ['当前 C = ' num2str(C, '%.4f')];
            
            state0 = [x0; y0; vx0; vy0];
            options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @(t,y) app.detectCrash(t,y,mu));
            
            try
                [t, Y, te, ye] = ode45(@(t,y) app.equations_of_motion(t,y,mu), [0 t_max], state0, options);
            catch ME
                uialert(app.UIFigure, ['积分错误: ' ME.message], '警告');
                return;
            end
            
            cla(app.AxOrbit);
            app.plot_background_on_axes(app.AxOrbit, mu, C);
            plot(app.AxOrbit, Y(:,1), Y(:,2), 'b-', 'LineWidth', 1.2);
            plot(app.AxOrbit, x0, y0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 5);
            
            if ~isempty(te)
                plot(app.AxOrbit, ye(:,1), ye(:,2), 'mx', 'MarkerSize', 10, 'LineWidth', 2);
            end
            
            title(app.AxOrbit, ['仿真完成 (C=' num2str(C, '%.3f') ')']);
            axis(app.AxOrbit, 'equal');
            grid(app.AxOrbit, 'on');
            app.RightTabs.SelectedTab = app.TabOrbit;
        end

        function RunExp1ButtonPushed(app, src, event)
            % 【修改点】时间统一为 10s
            mu_val = 0.1; C_target = 3.4; x0_list = [-0.2, 0, 0.2, 0.7];
            figure('Color', 'w', 'Name', '图12.5复现', 'NumberTitle', 'off');
            for i = 1:length(x0_list)
                x0 = x0_list(i);
                [vy0, success] = app.calculate_vy_from_c(x0, 0, C_target, mu_val);
                if ~success, continue; end
                options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @(t,y) app.detectCrash(t,y,mu_val));
                % 【修改点】[0 15] -> [0 10]
                [t, state] = ode45(@(t,y) app.equations_of_motion(t,y,mu_val), [0 10], [x0;0;0;vy0], options);
                subplot(2, 2, i); hold on;
                app.plot_background_on_axes(gca, mu_val, C_target);
                plot(state(:,1), state(:,2), 'b');
                title(['x_0 = ' num2str(x0)]); axis equal; xlim([-1.6 1.6]); ylim([-1.6 1.6]);
            end
        end

        function RunExp2ButtonPushed(app, src, event)
            % 【修改点】时间统一为 10s
            x0_fixed = 0.1; C_target = 3.4; mu_list = [0.03, 0.06, 0.1, 0.5];
            figure('Color', 'w', 'Name', '图12.6复现', 'NumberTitle', 'off');
            for i = 1:length(mu_list)
                mu_val = mu_list(i);
                [vy0, success] = app.calculate_vy_from_c(x0_fixed, 0, C_target, mu_val);
                if ~success, continue; end
                options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @(t,y) app.detectCrash(t,y,mu_val));
                % 【修改点】[0 18] -> [0 10]
                [t, state] = ode45(@(t,y) app.equations_of_motion(t,y,mu_val), [0 10], [x0_fixed;0;0;vy0], options);
                subplot(2, 2, i); hold on;
                app.plot_background_on_axes(gca, mu_val, C_target);
                plot(state(:,1), state(:,2), 'b');
                title(['\mu = ' num2str(mu_val)]); axis equal; xlim([-1.6 1.6]); ylim([-1.6 1.6]);
            end
        end
    end

    % App 初始化与布局代码
    methods (Access = public)
        function app = CRTBP_App_Final
            % 1. 创建 UIFigure
            app.UIFigure = uifigure('Position', [100 100 1100 700], 'Name', 'CRTBP 轨道仿真综合程序');
            
            % 主 Grid: 左侧固定宽，右侧自适应
            app.GridLayout = uigridlayout(app.UIFigure, [1 2]);
            app.GridLayout.ColumnWidth = {280, '1x'};
            
            % 2. 左侧面板容器
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.Title = '控制面板';
            app.LeftPanel.Layout.Column = 1;
            
            % 左侧滚动 Grid - 严格定义18行，防止控件被拉伸
            app.MainScrollLayout = uigridlayout(app.LeftPanel, [18 1]);
            app.MainScrollLayout.Scrollable = 'on';
            % 大部分行用 'fit' (适应内容高度)，PanelExp给一点高度，Input区紧凑
            app.MainScrollLayout.RowHeight = {100, 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 110, 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 'fit', 40, 40};
            
            % --- 填充左侧控件 (必须指定 Layout.Row) ---
            
            % Row 1: 批量实验面板
            app.PanelExp = uipanel(app.MainScrollLayout);
            app.PanelExp.Title = '批量实验 (独立窗口)';
            app.PanelExp.Layout.Row = 1;
            peGrid = uigridlayout(app.PanelExp, [2 1]);
            peGrid.RowHeight = {'fit', 'fit'};
            app.BtnExp1 = uibutton(peGrid, 'Text', '复现图 12.5 (变 x0)', 'ButtonPushedFcn', @app.RunExp1ButtonPushed);
            app.BtnExp2 = uibutton(peGrid, 'Text', '复现图 12.6 (变 mu)', 'ButtonPushedFcn', @app.RunExp2ButtonPushed);
            
            % Row 2: 分隔符
            app.LabelSep = uilabel(app.MainScrollLayout, 'Text', '------ 单次交互仿真 ------', 'HorizontalAlignment', 'center');
            app.LabelSep.Layout.Row = 2;
            
            % Row 3-4: Mu
            app.LabelMu = uilabel(app.MainScrollLayout, 'Text', '质量比 (μ):');
            app.LabelMu.Layout.Row = 3;
            app.EditMu = uieditfield(app.MainScrollLayout, 'numeric', 'Value', 0.1);
            app.EditMu.Layout.Row = 4;
            
            % Row 5-6: X0
            app.LabelX0 = uilabel(app.MainScrollLayout, 'Text', '初始位置 X0:');
            app.LabelX0.Layout.Row = 5;
            app.EditX0 = uieditfield(app.MainScrollLayout, 'numeric', 'Value', -0.2);
            app.EditX0.Layout.Row = 6;
            
            % Row 7-8: Y0
            app.LabelY0 = uilabel(app.MainScrollLayout, 'Text', '初始位置 Y0:');
            app.LabelY0.Layout.Row = 7;
            app.EditY0 = uieditfield(app.MainScrollLayout, 'numeric', 'Value', 0);
            app.EditY0.Layout.Row = 8;
            
            % Row 9: 初始速度模式面板
            app.PanelC = uipanel(app.MainScrollLayout);
            app.PanelC.Title = '初始速度模式';
            app.PanelC.Layout.Row = 9;
            pcGrid = uigridlayout(app.PanelC, [3 1]);
            pcGrid.RowHeight = {'fit', 'fit', 'fit'};
            app.CheckBoxAutoV = uicheckbox(pcGrid, 'Text', '通过能量 C 反算 Vy', 'ValueChangedFcn', @app.CheckBoxAutoVValueChanged);
            gC = uigridlayout(pcGrid, [1 2]); gC.Padding=[0 0 0 0];
            uilabel(gC, 'Text', '目标 C:');
            app.EditTargetC = uieditfield(gC, 'numeric', 'Value', 3.4, 'Enable', 'off');
            
            % Row 10-11: Vx0
            app.LabelVx0 = uilabel(app.MainScrollLayout, 'Text', '初始速度 Vx0:');
            app.LabelVx0.Layout.Row = 10;
            app.EditVx0 = uieditfield(app.MainScrollLayout, 'numeric', 'Value', 0);
            app.EditVx0.Layout.Row = 11;
            
            % Row 12-13: Vy0
            app.LabelVy0 = uilabel(app.MainScrollLayout, 'Text', '初始速度 Vy0:');
            app.LabelVy0.Layout.Row = 12;
            app.EditVy0 = uieditfield(app.MainScrollLayout, 'numeric', 'Value', 2.5);
            app.EditVy0.Layout.Row = 13;
            
            % Row 14-15: TMax
            app.LabelTMax = uilabel(app.MainScrollLayout, 'Text', '仿真时间 T:');
            app.LabelTMax.Layout.Row = 14;
            % 【修改点】默认值改为 10
            app.EditTMax = uieditfield(app.MainScrollLayout, 'numeric', 'Value', 10);
            app.EditTMax.Layout.Row = 15;
            
            % Row 16: 当前C显示
            app.LabelRealC = uilabel(app.MainScrollLayout, 'Text', '当前 C = N/A', 'FontWeight', 'bold', 'FontColor', 'b');
            app.LabelRealC.Layout.Row = 16;
            
            % Row 17: 运行按钮
            app.BtnSimulate = uibutton(app.MainScrollLayout, 'Text', '运行当前仿真', ...
                'ButtonPushedFcn', @app.SimulateButtonPushed, 'BackgroundColor', [0.6 0.8 1], 'FontWeight', 'bold');
            app.BtnSimulate.Layout.Row = 17;
            
            % Row 18: 势能按钮
            app.BtnPlotEnergy = uibutton(app.MainScrollLayout, 'Text', '查看势能曲面', ...
                'ButtonPushedFcn', @app.PlotEnergyButtonPushed);
            app.BtnPlotEnergy.Layout.Row = 18;
            
            % 3. 右侧绘图区
            app.RightTabs = uitabgroup(app.GridLayout);
            app.RightTabs.Layout.Column = 2;
            
            % Tab 1: 轨道 (内部使用 Grid Layout 以自适应)
            app.TabOrbit = uitab(app.RightTabs, 'Title', '轨道视图');
            orbitGrid = uigridlayout(app.TabOrbit, [1 1]); % 1行1列，占满
            app.AxOrbit = uiaxes(orbitGrid);
            app.AxOrbit.Layout.Row = 1;
            app.AxOrbit.Layout.Column = 1;
            
            % Tab 2: 势能 (内部使用 Grid Layout 以自适应)
            app.TabEnergy = uitab(app.RightTabs, 'Title', '势能视图');
            energyGrid = uigridlayout(app.TabEnergy, [1 1]); % 1行1列，占满
            app.AxEnergy = uiaxes(energyGrid);
            app.AxEnergy.Layout.Row = 1;
            app.AxEnergy.Layout.Column = 1;
        end
    end
end