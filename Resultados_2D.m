%% Tesis en 2D
%% Parte en 2D
close all
clear 
clc
% Nombres de los archivos de volumen

 VASOS = {'1.1.png', '2.1.png', '3.1.png', '4.1.png'};
%  VASOS = {'1.2.png', '2.2.png', '3.2.png', '4.2.png'};
%  VASOS = {'1.3.png', '2.3.png', '3.3.png', '4.3.png'};
%  VASOS = {'1.4.png', '2.4.png', '3.4.png', '4.4.png'};
%  VASOS = {'1.5.png', '2.5.png', '3.5.png', '4.5.png'};

letras = ["a", "b", "c", "d"];

jaccard_indices = [];
dice_indices = [];
sensibilidades = [];
especificidades = [];



% Crear una figura para las imágenes de vasos
figure_vasos = figure('Name', 'Imágenes de Vasos');
% Crear una figura para las imágenes de label_tracking
figure_label_tracking = figure('Name', 'Imágenes de Label Tracking');
figure_tendencia = figure('Name', 'Gráficas tendencia');
figure_deteccion = figure('Name', 'Imágenes de detección');
segmentacion = figure('Name', 'Imágenes de segmentación');

% JT = ["4.2.1.png","4.2.2.png","4.2.3.png","4.2.4.png"];
 JT = ["1.1.GT.png","2.1.GT.png","3.1.GT.png","4.1.GT.png"];
% JT = ["1.2.GT.png","2.2.GT.png","3.2.GT.png","4.2.GT.png"];
% JT = ["1.3.GT.png","2.3.GT.png","3.3.GT.png","4.3.GT.png"];
% JT = ["1.4.GT.png","2.4.GT.png","3.4.GT.png","4.4.GT.png"];
% JT = ["1.5.GT.png","2.5.GT.png","3.5.GT.png","4.5.GT.png"];


jac = 1;

% Iterar sobre cada archivo de volumen
for idx = 1:1%numel(VASOS)

    vasos = imread(VASOS{idx});
    % Generar el esqueleto de una imagen binaria
    vasos_gris = rgb2gray(vasos);
    BW = imbinarize(vasos_gris);
    skeleton = bwskel(BW);
    
    mascara_aneurismas = zeros(size(BW));

    % Inicializar una matriz para almacenar las etiquetas de seguimiento de cada píxel del esqueleto
    label_tracking = zeros(size(skeleton));

    % Definir los desplazamientos en la vecindad 3x3
    dr = [-1, -1, -1, 0, 0, 1, 1, 1];
    dc = [-1, 0, 1, -1, 1, -1, 0, 1];

    % Inicializar el contador de etiquetas
    current_label = 1;

    % Recorrer cada píxel en el esqueleto
    for i = 1:size(skeleton, 1)
        for j = 1:size(skeleton, 2)
            % Verificar si el píxel actual pertenece al esqueleto y aún no ha sido etiquetado
            if skeleton(i, j) && label_tracking(i, j) == 0
                % Inicializar la cola para propagar la etiqueta
                queue = [i, j];
                % Mientras haya elementos en la cola
                while ~isempty(queue)
                    % Extraer el primer elemento de la cola
                    current_pixel = queue(1, :);
                    queue(1, :) = [];
                    % Asignar la etiqueta actual al píxel actual
                    label_tracking(current_pixel(1), current_pixel(2)) = current_label;
                    % Incrementar el contador de etiquetas
                    current_label = current_label + 1;
                    % Recorrer la vecindad 3x3 del píxel actual
                    for k = 1:numel(dr)
                        r = current_pixel(1) + dr(k);
                        c = current_pixel(2) + dc(k);
                        % Verificar si el píxel vecino está dentro de la imagen y pertenece al esqueleto y no ha sido etiquetado
                        if r >= 1 && r <= size(skeleton, 1) && c >= 1 && c <= size(skeleton, 2) && skeleton(r, c) && label_tracking(r, c) == 0
                            % Asignar la etiqueta actual al píxel vecino
                            label_tracking(r, c) = current_label;
                            % Agregar el píxel vecino a la cola
                            queue = [queue; r, c];  
                            break
                        end
                    end
                end
            end
        end
    end


    % Mostrar las imágenes de vasos en subplots
    figure(figure_vasos);
    subplot(1, 4, idx);
    imshow(vasos);
    title(sprintf('%c', letras(idx)));
    %%%imwrite(vasos, sprintf('vasos_%c.tiff', letras(idx)), 'Resolution', 400);

    
    % Mostrar las imágenes de label_tracking en subplots
    figure(figure_label_tracking);
    subplot(1, 4, idx);
    imshow(label2rgb(label_tracking, 'turbo'));
    title(sprintf('%c', letras(idx)));
    
    figure;
    imshow(label2rgb(label_tracking, 'turbo'));
    title(sprintf('%c', letras(idx)));
    
    
%% Gráfica scatter
% Crear una nueva figura
figure;
hold on;

% Graficar cada segmento con su propio color
[r_tracking, c_tracking] = find(label_tracking > 0);
labels = label_tracking(label_tracking > 0);
unique_labels = unique(labels);
colors = turbo(length(unique_labels));

% Ordenar los puntos por etiqueta
for i = 1:length(unique_labels)
    % Encontrar todos los puntos con esta etiqueta
    idx_label = find(labels == unique_labels(i));
    r = max(r_tracking) - r_tracking(idx_label);
    c = c_tracking(idx_label);
    
    % Ordenar los puntos para una mejor conexión
    [~, order] = sort(c);
    r = r(order);
    c = c(order);
    
    % Graficar puntos y líneas
    scatter(c, r, 40, colors(i,:), 'filled', 'DisplayName', sprintf('Segment %d', i));
    plot(c, r, '-', 'Color', colors(i,:), 'LineWidth', 1);
    
    drawnow;  % Forzar la actualización de la figura en cada iteración

    pause(0.005);  % Controlar la velocidad de actualización (ajustar según sea necesario)

end

% Configurar el aspecto de la gráfica
%grid on;
axis equal;
xlabel('X');
ylabel('Y');
title(sprintf('%c', letras(idx)));

% Ajustar los límites de los ejes
xlim([min(c_tracking)-10 max(c_tracking)+10]);
ylim([0 max(max(r_tracking))+10]);
% 
%     set(gcf, 'Units', 'Inches');
%     set(gcf, 'Position', [0 0 6 4]); % [x y width height]
%   exportgraphics(gcf, 'Aneurisma2D_skeleton.pdf', 'ContentType', 'vector')

%% Video
%% Gráfica scatter con ejes fijos y guardado en GIF
%% Gráfica scatter con ejes fijos y guardado en video
figure;
hold on;
grid on;


% Definir el nombre y formato del video
video_filename = 'animacion.mp4';  % Puedes usar .avi o .mp4
v = VideoWriter(video_filename, 'MPEG-4');  % Usar 'Motion JPEG AVI' para .avi
v.FrameRate = 20;  % Ajustar la velocidad del video
open(v);

% Obtener los límites de los ejes antes del bucle
[r_tracking, c_tracking] = find(label_tracking > 0);
x_min = min(c_tracking) - 10;
x_max = max(c_tracking) + 10;
y_min = 0;
y_max = max(max(r_tracking)) + 10;

% Configurar los límites de los ejes antes de entrar al bucle
xlim([x_min, x_max]);
ylim([y_min, y_max]);
axis equal;

% Etiquetas y título
xlabel('X');
ylabel('Y');

% Obtener etiquetas únicas y colores
labels = label_tracking(label_tracking > 0);
unique_labels = unique(labels);
colors = turbo(length(unique_labels));

for i = 1:length(unique_labels)
    % Encontrar todos los puntos con esta etiqueta
    idx_label = find(labels == unique_labels(i));
    r = max(r_tracking) - r_tracking(idx_label);
    c = c_tracking(idx_label);
    
    % Ordenar los puntos para una mejor conexión
    [~, order] = sort(c);
    r = r(order);
    c = c(order);
    
    % Graficar puntos y líneas
    scatter(c, r, 40, colors(i,:), 'filled', 'DisplayName', sprintf('Segment %d', i));
    plot(c, r, '-', 'Color', colors(i,:), 'LineWidth', 1);
    
    drawnow;  % Forzar la actualización de la figura en cada iteración

    % Capturar el frame y guardarlo en el video
    frame = getframe(gcf);
    writeVideo(v, frame);
    
    pause(0.005);  % Controlar la velocidad de actualización
end

% Cerrar el archivo de video
close(v);
disp(['Video guardado como: ', video_filename]);




    %% Medición diámetro de vasos 
    % Inicializar el vector para almacenar el número de píxeles en cada medición
    pixeles_por_medicion = [];

    % Inicializar el vector para almacenar los índices de las mediciones con cambio mayor o igual al 30%
    indices_tasa_cambio = [];
    num_tasa_cambio = [];
    pixeles_vaso_vector = [];

    % Inicializar la imagen con todas las líneas rojas perpendiculares
    imagen_con_lineas_cambio = vasos_gris;
    imagen_seguimiento = vasos_gris;
    solo_lineas = zeros(size(vasos_gris));
    solo_puntos = zeros(size(vasos_gris));

    % Definir la variable delta_medicion
    delta_medicion = 10 + 15;


    label = 1;
    while label <= current_label
        % Encontrar las coordenadas de los puntos 1 y 3
        [filas_punto1, columnas_punto1] = find(label_tracking == label, 1, 'first');
        [filas_punto2, columnas_punto2] = find(label_tracking == label + delta_medicion - 6, 1, 'first');

        % Verificar si se encontraron ambos puntos
        if ~isempty(filas_punto1) && ~isempty(filas_punto2)
            punto1 = [columnas_punto1, filas_punto1];
            punto2 = [columnas_punto2, filas_punto2];

            % Calcular la distancia entre los dos puntos
            distancia = sqrt((punto2(1) - punto1(1))^2 + (punto2(2) - punto1(2))^2);

            % Verificar la distancia antes de continuar
            if distancia <= delta_medicion
                % Calcular la pendiente entre los dos puntos
                pendiente = (punto2(2) - punto1(2)) / (punto2(1) - punto1(1));

                % Calcular las coordenadas para la línea perpendicular en el segundo punto
                longitud_linea = 50;
                angulo_perpendicular = atan(-1 / pendiente);
                x_inicio_perpendicular = punto2(1) - longitud_linea / 2 * cos(angulo_perpendicular);
                y_inicio_perpendicular = punto2(2) - longitud_linea / 2 * sin(angulo_perpendicular);
                x_fin_perpendicular = punto2(1) + longitud_linea / 2 * cos(angulo_perpendicular);
                y_fin_perpendicular = punto2(2) + longitud_linea / 2 * sin(angulo_perpendicular);

                % Dibujar la línea entre los dos puntos

                % Contar píxeles en ambas direcciones desde el punto2 hasta encontrar un píxel negro
                pixeles_vaso = 0;
                % Búsqueda hacia adelante
                t = 0;
                while true
                    x_actual = round(punto2(1) + t * cos(angulo_perpendicular));
                    y_actual = round(punto2(2) + t * sin(angulo_perpendicular));
                    % Verificar que el punto actual esté dentro de los límites de la imagen
                    if x_actual <= 0 || x_actual > size(BW, 2) || y_actual <= 0 || y_actual > size(BW, 1)
                        break;
                    end
                    % Verificar si se encuentra un cero
                    if BW(y_actual, x_actual) == 0
                        break;
                    end
                    t = t + 1;
                    pixeles_vaso = pixeles_vaso + 1;
                end

                % Búsqueda hacia atrás
                t = 0;
                while true
                    x_actual = round(punto2(1) - t * cos(angulo_perpendicular));
                    y_actual = round(punto2(2) - t * sin(angulo_perpendicular));
                    % Verificar que el punto actual esté dentro de los límites de la imagen
                    if x_actual <= 0 || x_actual > size(BW, 2) || y_actual <= 0 || y_actual > size(BW, 1)
                        break;
                    end
                    % Verificar si se encuentra un cero
                    if BW(y_actual, x_actual) == 0
                        break;
                    end
                    t = t + 1;
                    pixeles_vaso = pixeles_vaso + 1;
                end

                % Guardar el resultado de la medición actual
                pixeles_por_medicion = [pixeles_por_medicion, pixeles_vaso];
                pixeles_vaso_vector = [pixeles_vaso_vector, [x_inicio_perpendicular; y_inicio_perpendicular; x_fin_perpendicular; y_fin_perpendicular; columnas_punto2; filas_punto2]];
                indices_tasa_cambio = [indices_tasa_cambio, [filas_punto2; columnas_punto2; angulo_perpendicular]];


            end
        end

        % Incrementar label para el siguiente conjunto de puntos
        label = label + 1;


    end

    hold off;

    
    
% % Preparar la figura
% figure;
% imshow(vasos_gris, []);
% hold on;
% 
% % Inicializar las variables
% pixeles_por_medicion = [];
% pixeles_vaso_vector = [];
% indices_tasa_cambio = [];
% 
% % Crear manejadores para las gráficas
% h_line = plot(NaN, NaN, 'r-', 'LineWidth', 2); % Línea roja entre punto 1 y punto 2
% h_perpendicular = plot(NaN, NaN, 'b-', 'LineWidth', 1.5); % Línea azul perpendicular
% 
% label = 1;
% while label <= current_label
%     % Encontrar las coordenadas de los puntos 1 y 2
%     [filas_punto1, columnas_punto1] = find(label_tracking == label, 1, 'first');
%     [filas_punto2, columnas_punto2] = find(label_tracking == label + delta_medicion - 6, 1, 'first');
%     
%     if ~isempty(filas_punto1) && ~isempty(filas_punto2)
%         punto1 = [columnas_punto1, filas_punto1];
%         punto2 = [columnas_punto2, filas_punto2];
%         
%         % Calcular distancia
%         distancia = sqrt((punto2(1) - punto1(1))^2 + (punto2(2) - punto1(2))^2);
%         
%         if distancia <= delta_medicion
%             % Dibujar la línea entre los puntos 1 y 2
%             set(h_line, 'XData', [punto1(1), punto2(1)], ...
%                         'YData', [punto1(2), punto2(2)]);
% 
%             % Calcular pendiente y línea perpendicular en punto 2
%             pendiente = (punto2(2) - punto1(2)) / (punto2(1) - punto1(1));
%             angulo_perpendicular = atan(-1 / pendiente);
%             longitud_linea = 30;
% 
%             x_inicio_perpendicular = punto2(1) - longitud_linea / 2 * cos(angulo_perpendicular);
%             y_inicio_perpendicular = punto2(2) - longitud_linea / 2 * sin(angulo_perpendicular);
%             x_fin_perpendicular = punto2(1) + longitud_linea / 2 * cos(angulo_perpendicular);
%             y_fin_perpendicular = punto2(2) + longitud_linea / 2 * sin(angulo_perpendicular);
% 
%             % Actualizar la línea perpendicular en la gráfica
%             set(h_perpendicular, 'XData', [x_inicio_perpendicular, x_fin_perpendicular], ...
%                                  'YData', [y_inicio_perpendicular, y_fin_perpendicular]);
% 
%             % Pausar para simular tiempo real
%             pause(0.1);
%         end
%     end
%     
%     label = label + 1;
% end
% 
% hold off;


    %% Calcular tendencia después del ciclo
    % Calcular la tendencia con polyfit
    grado = 1;  
    p = polyfit(1:numel(pixeles_por_medicion), pixeles_por_medicion, grado);

    % Forzar la pendiente a ser 0
    p = [0, p(2)];

    umbral = round(p(2));

    % Mostrar la tendencia
    fprintf('La pendiente de la tendencia es: %.4f\n', p(1));
    fprintf('El intercepto de la tendencia es: %.4f\n', p(2));

    % Aplicar condición sobre pixeles_vaso usando intercepto
    for i = 1:numel(pixeles_por_medicion)
        if pixeles_por_medicion(i) > umbral + 7

          solo_puntos(indices_tasa_cambio(1,i),indices_tasa_cambio(2,i)) = 1;
        end
    end
% 
%     % Graficar los diámetros
%     fig5 = figure;
%     plot(pixeles_por_medicion, '-o','Color', 'b','DisplayName', 'Diameters','LineWidth', 1);
%     xlabel('Label number');
%     ylabel('Diameter (pixels)');
%     %title('Diámetro en pixeles vs. Número de etiqueta');
%     grid on;
%     % Ajustar los ejes para que comiencen en 0
%     xlim([0, length(pixeles_por_medicion) + 1]);
%     ylim([0, max(pixeles_por_medicion) + 1]);
%     % Añadir una línea roja con el valor del intercepto
%     yline(p(2), 'Color','r','DisplayName','Threshold', 'LineWidth', 2);
%     legend('show');
%     filename = sprintf('tendencias_%d.tiff', idx);
%     print(fig5, filename, '-dsvg');
% %     
%     
%     % Guardar la figura con una resolución de 400 DPI
%     filename = sprintf('tendencias_%d.tiff', idx);
%     print(filename, '-dtiff', '-r400');  % Guardar como TIFF con 400 DPI

    
    % Mostrar las imágenes de label_tracking en subplots
    figure(figure_tendencia);
    subplot(2, 2, idx);
    plot(pixeles_por_medicion, '-o','Color', '#4DBEEE','DisplayName', 'Diameters','LineWidth', 0.5);
    xlabel('Label number');
    ylabel('Diameter (pixels)');
    %title('Diámetro en pixeles vs. Número de etiqueta');
    grid on;
    % Ajustar los ejes para que comiencen en 0
    xlim([0, length(pixeles_por_medicion) + 1]);
    ylim([0, max(pixeles_por_medicion) + 1]);
    % Añadir una línea roja con el valor del intercepto
    yline(p(2), 'Color','#A2142F','DisplayName','Threshold', 'LineWidth', 2);
    legend('show');
    title(sprintf('%c', letras(idx)));
    
     % Mostrar las imágenes de label_tracking en subplots
    figure;
    plot(pixeles_por_medicion, '-o','Color', '#4DBEEE','DisplayName', 'Diámetros','LineWidth', 0.5);
    xlabel('Número de etiqueta');
    ylabel('Diámetro (píxeles)');
    %title('Diámetro en pixeles vs. Número de etiqueta');
    grid on;
    % Ajustar los ejes para que comiencen en 0
    xlim([0, length(pixeles_por_medicion) + 1]);
    ylim([0, max(pixeles_por_medicion) + 1]);
    % Añadir una línea roja con el valor del intercepto
    yline(p(2), 'Color','#A2142F','DisplayName','Umbral', 'LineWidth', 2);
    legend('show');
    
%     set(gcf, 'Units', 'Inches');
%     set(gcf, 'Position', [0 0 6 4]); % [x y width height]
%     exportgraphics(gcf, 'Aneurisma2D_tendencia.pdf', 'ContentType', 'vector')



    figure;
    plot(pixeles_por_medicion, '-o','Color', '#4DBEEE','DisplayName', 'Diameters','LineWidth', 0.5);
    xlabel('Label number');
    ylabel('Diameter (pixels)');
    %title('Diámetro en pixeles vs. Número de etiqueta');
    grid on;
    % Ajustar los ejes para que comiencen en 0
    xlim([0, length(pixeles_por_medicion) + 1]);
    ylim([0, max(pixeles_por_medicion) + 1]);
    % Añadir una línea roja con el valor del intercepto
    yline(p(2), 'Color','#A2142F','DisplayName','Threshold ', 'LineWidth', 2);
    legend('show');
    
     set(gcf, 'Units', 'Inches');
     set(gcf, 'Position', [0 0 6 4]); % [x y width height]
     exportgraphics(gcf, 'InglesAneurisma2D_tendencia.pdf', 'ContentType', 'vector')

    %% Grupos cortados (puntos)

diametros = [];
imagen_con_lineas = vasos;

componentes_conectados_puntos = bwconncomp(solo_puntos);

% Filtrado 
filtrado_objetos = bwareaopen(solo_puntos, umbral);

CC_filtrado = bwconncomp(filtrado_objetos);

% Obtener propiedades de las regiones conectadas
propiedades_puntos = regionprops(CC_filtrado, 'Centroid', 'PixelList', 'Circularity');
figure
%figure(figure_deteccion);
%subplot(2, 2, idx);
imshow(vasos);
hold on;
%title(sprintf(' %c', letras(idx)));

% Añadir los textos "Punto 1" y "Punto 2" y las líneas entre ellos
for i = 1:CC_filtrado.NumObjects
    pixel_list = propiedades_puntos(i).PixelList;
    punto1 = pixel_list(1, :);
    punto2 = pixel_list(end, :);

    % Dibujar la línea en la imagen
    plot([punto1(1), punto2(1)], [punto1(2), punto2(2)], 'Color', 'b', 'LineWidth', 2);
    plot(punto1(1), punto1(2), 'ro', 'MarkerSize', 2, 'Color', 'r', 'LineWidth', 1);
    plot(punto2(1), punto2(2), 'bo', 'MarkerSize', 2, 'Color', 'r', 'LineWidth', 1);
end

hold off;





%% Segmentación con líneas de color 
% Crear una copia de la imagen original con las líneas negras
image_with_lines = vasos;

% Asegurarse de que la imagen sea RGB para poder dibujar líneas de colores
if size(image_with_lines, 3) == 1
    image_with_lines = repmat(image_with_lines, [1, 1, 3]);
end

% Mostrar la imagen original (opcional)
figure;
imshow(image_with_lines);
hold on; % Importante: Mantener la figura abierta para dibujar encima

% Añadir las líneas verdes a la imagen
for i = 1:CC_filtrado.NumObjects
    pixel_list = propiedades_puntos(i).PixelList;
    punto1 = pixel_list(1, :);
    punto2 = pixel_list(end, :);

    % Encontrar las coordenadas basadas en tus variables
    indice1 = find(pixeles_vaso_vector(5,:) == punto1(1) & pixeles_vaso_vector(6,:) == punto1(2));
    x1 = round(pixeles_vaso_vector(1,indice1))-3;
    y1 = round(pixeles_vaso_vector(2,indice1));
    x2 = round(pixeles_vaso_vector(3,indice1))-3;
    y2 = round(pixeles_vaso_vector(4,indice1));

    % Dibujar la línea verde
    line([x1, x2], [y1, y2], 'Color', [1, 0, 0], 'LineWidth', 2); % Verde, grosor 2

    indice2 = find(pixeles_vaso_vector(5,:) == punto2(1) & pixeles_vaso_vector(6,:) == punto2(2));
    x1 = round(pixeles_vaso_vector(1,indice2))+3;
    y1 = round(pixeles_vaso_vector(2,indice2));
    x2 = round(pixeles_vaso_vector(3,indice2))+3;
    y2 = round(pixeles_vaso_vector(4,indice2));

    % Dibujar la línea verde
    line([x1, x2], [y1, y2], 'Color', [1, 0, 0], 'LineWidth', 2); % Verde, grosor 2
end

hold off; % Liberar la figura

% Mostrar la imagen con líneas verdes (ya se mostró antes, pero se actualizó con las líneas)
figure; % No es necesario crear una nueva figura
imshow(image_with_lines); % Ya se mostró antes con las líneas dibujadas


exportgraphics(gcf, 'Aneurisma2D_segmentacion.pdf', 'ContentType', 'vector')




%%

% Crear una copia de la imagen original con las líneas negras
image_with_lines = vasos;

% Asegurarse de que la imagen sea RGB para poder dibujar líneas de colores
if size(image_with_lines, 3) == 1
    image_with_lines = repmat(image_with_lines, [1, 1, 3]);
end

% Añadir las líneas negras a la imagen
for i = 1:CC_filtrado.NumObjects
    pixel_list = propiedades_puntos(i).PixelList;
    punto1 = pixel_list(1, :);
    punto2 = pixel_list(end, :);

    % Encontrar las coordenadas basadas en tus variables
    indice1 = find(pixeles_vaso_vector(5,:) == punto1(1) & pixeles_vaso_vector(6,:) == punto1(2));
    x1 = round(pixeles_vaso_vector(1,indice1))-3;
    y1 = round(pixeles_vaso_vector(2,indice1));
    x2 = round(pixeles_vaso_vector(3,indice1))-3;
    y2 = round(pixeles_vaso_vector(4,indice1));
    image_with_lines = insert_line(image_with_lines, [x1, y1], [x2, y2]);

    indice2 = find(pixeles_vaso_vector(5,:) == punto2(1) & pixeles_vaso_vector(6,:) == punto2(2));
    x1 = round(pixeles_vaso_vector(1,indice2))+3;
    y1 = round(pixeles_vaso_vector(2,indice2));
    x2 = round(pixeles_vaso_vector(3,indice2))+3;
    y2 = round(pixeles_vaso_vector(4,indice2));
    image_with_lines = insert_line(image_with_lines, [x1, y1], [x2, y2]);
end

% Mostrar la imagen con líneas negras
figure;
imshow(image_with_lines);





   % Convertir la imagen a binaria
BW_with_lines = imbinarize(rgb2gray(image_with_lines));

% Calcular los componentes conectados
CC_separado = bwconncomp(BW_with_lines);

% Obtener las propiedades de los objetos conectados
propiedades_CC_separado = regionprops(CC_separado, 'PixelList', 'Circularity');

% Inicializar el vector de diámetros
diametros = [];

for i = 1:length(propiedades_CC_separado)
    % Obtener las coordenadas del primer pixel del objeto actual
    pixel_list = propiedades_CC_separado(i).PixelList;
    punto1 = pixel_list(1, :);
    % Obtener las coordenadas del último pixel del objeto actual
    punto2 = pixel_list(end, :);
    % Calcular la distancia euclidiana entre el primer y último punto
    distancia = sqrt((punto2(1) - punto1(1))^2 + (punto2(2) - punto1(2))^2);
    % Guardar la distancia en la variable diámetros
    diametros = [diametros, distancia];
end


    
        % Iterar sobre cada objeto en CC_separado
    for i = 1:CC_separado.NumObjects
        % Obtener el valor de circularidad del objeto actual
        circularidad = propiedades_CC_separado(i).Circularity;
        area = length(propiedades_CC_separado(i).PixelList);
        
%         prueba = false(size(BW_with_lines));
%             prueba(CC_separado.PixelIdxList{i}) = true;
%             prueba = imfill(prueba,"holes");
%             figure;
%             imshow(prueba);
%             title(['Objeto ', num2str(i)]);
%             % Superponer texto en la imagen con información sobre circularidad, área y diámetro
%             texto = sprintf('Circularity: %.2f\nArea: %d pixels\nDiameter: %.2f pixels', circularidad, area, diametros(i));
%             text(10, 450, texto, 'Color', 'white', 'FontSize', 12, 'FontWeight', 'bold');
%             

        % Filtrar objetos con circularidad mayor o igual a 0.8
        if circularidad >= 0.85 && circularidad <= 1.1 && area > umbral && diametros(i) > umbral + 3 
            % Crear una imagen solo con el objeto actual
            objeto_actual = false(size(BW_with_lines));
            objeto_actual(CC_separado.PixelIdxList{i}) = true;

            objeto_actual = imfill(objeto_actual,"holes");
            
            mascara_aneurismas = mascara_aneurismas + objeto_actual;

           % Mostrar el objeto actual en una nueva figura con el valor de circularidad en el título
            figure;
            imshow(objeto_actual);
            title(['Objeto ', num2str(i)]);
            % Superponer texto en la imagen con información sobre circularidad, área y diámetro
            texto = sprintf('Área: %d píxeles\nDiámetro: %.2f píxeles', area, diametros(i));
            text(10, 450, texto, 'Color', 'white', 'FontSize', 12, 'FontWeight', 'bold');
            
            set(gcf, 'Units', 'Inches');
            set(gcf, 'Position', [0 0 6 4]); % [x y width height]
            exportgraphics(gcf, 'Aneurisma2D_caracterizado.pdf', 'ContentType', 'vector')
            
            
            
   
            % % Coordenadas de recorte específicas para cada imagen
            % [x, y, width, height] en píxeles
            crop_coords = [
                241, 303, 100, 100;  % Coordenadas para idx == 1
                251, 228, 100, 100;  % Coordenadas para idx == 2
                294, 59, 100, 100;  % Coordenadas para idx == 3
                218, 154, 100, 100   % Coordenadas para idx == 4
            ];

            if i == 10 && idx == 1
                figure(segmentacion);
                subplot(1, 4, idx);
                cropped_image = imcrop(objeto_actual, crop_coords(idx, :)); % Recorta la imagen
                imshow(cropped_image);
                title(sprintf(' %c', letras(idx)));
                imwrite(cropped_image, sprintf('aneu_%c.tiff', letras(idx)), 'Resolution', 400);
                hold on;
            end
            if i == 25 && idx == 2
                figure(segmentacion);
                subplot(1, 4, idx);
                cropped_image = imcrop(objeto_actual, crop_coords(idx, :)); % Recorta la imagen
                imshow(cropped_image);
                title(sprintf(' %c', letras(idx)));
                imwrite(cropped_image, sprintf('aneu_%c.tiff', letras(idx)), 'Resolution', 400);
                hold on;
            end
            if i == 18 && idx == 3
                figure(segmentacion);
                subplot(1, 4, idx);
                cropped_image = imcrop(objeto_actual, crop_coords(idx, :)); % Recorta la imagen
                imshow(cropped_image);
                title(sprintf(' %c', letras(idx)));
                imwrite(cropped_image, sprintf('aneu_%c.tiff', letras(idx)), 'Resolution', 400);
                hold on;
            end
            if i == 30 && idx == 4
                figure(segmentacion);
                subplot(1, 4, idx);
                cropped_image = imcrop(objeto_actual, crop_coords(idx, :)); % Recorta la imagen
                imshow(cropped_image);
                title(sprintf(' %c', letras(idx)));
                imwrite(cropped_image, sprintf('aneu_%c.tiff', letras(idx)), 'Resolution', 400);
                hold on;
            end
            
        end
    end
    %Métricas de evaluación
    aneu = imread(JT(jac));
    jac = jac+1;
    aneu = rgb2gray(aneu);
    gt_mask = imbinarize(aneu);
    mascara_aneurismas = imbinarize(mascara_aneurismas);

    jaccard_index = jaccard(mascara_aneurismas, gt_mask);
    dice_index = dice(mascara_aneurismas, gt_mask);

    % Almacenar los valores en los arrays
    jaccard_indices = [jaccard_indices; jaccard_index];
    dice_indices = [dice_indices; dice_index];    
    
    

end

% % Guardar la figura completa de vasos con una resolución de 400 dpi
% print(figure_vasos, 'vasos_completo', '-dtiff', '-r400');
% % Guardar la figura completa de label tracking con una resolución de 400 dpi
% print(figure_label_tracking, 'label_tracking_completo', '-dtiff', '-r400');
% % Guardar la figura completa de label tracking con una resolución de 400 dpi
% print(figure_tendencia, 'tendencias', '-dtiff', '-r400');
% % Guardar la figura completa de label tracking con una resolución de 400 dpi
% print(figure_deteccion, 'deteccion', '-dtiff', '-r400');
% print(segmentacion, 'segmentacion', '-dtiff', '-r400');

% imagen_original = imread('segmentacion.tif');
% imwrite(imagen_original, 'segmentacion1.tif', 'Resolution', 400);

% Función para dibujar líneas en la imagen
function img = insert_line(img, point1, point2)
    lineWidth = 2;
    color = [0, 0, 0]; % Negro
    
    % Dibujar la línea utilizando Bresenham's line algorithm
    points = bresenham(point1, point2);
    for i = 1:size(points, 1)
        for dx = -floor(lineWidth/2):floor(lineWidth/2)
            for dy = -floor(lineWidth/2):floor(lineWidth/2)
                x = points(i, 1) + dx;
                y = points(i, 2) + dy;
                if x > 0 && y > 0 && x <= size(img, 2) && y <= size(img, 1)
                    img(y, x, :) = color;
                end
            end
        end
    end
end

% Algoritmo de Bresenham para calcular los puntos de una línea
function points = bresenham(p1, p2)
    x1 = p1(1);
    y1 = p1(2);
    x2 = p2(1);
    y2 = p2(2);
    dx = abs(x2 - x1);
    dy = abs(y2 - y1);
    steep = abs(dy) > abs(dx);
    if steep
        [x1, y1] = swap(x1, y1);
        [x2, y2] = swap(x2, y2);
    end
    if x1 > x2
        [x1, x2] = swap(x1, x2);
        [y1, y2] = swap(y1, y2);
    end
    dx = x2 - x1;
    dy = abs(y2 - y1);
    error = dx / 2;
    ystep = -1;
    if y1 < y2
        ystep = 1;
    end
    y = y1;
    points = [];
    for x = x1:x2
        if steep
            points = [points; y, x];
        else
            points = [points; x, y];
        end
        error = error - dy;
        if error < 0
            y = y + ystep;
            error = error + dx;
        end
    end
end

% Función auxiliar para intercambiar valores
function [a, b] = swap(a, b)
    temp = a;
    a = b;
    b = temp;
end