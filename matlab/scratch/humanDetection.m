peopleDetector = vision.PeopleDetector;
rd = rosData;
figure;
while(1)
    I = rd.getRGBimage;
    imshow(I)
    [bboxes,scores] = peopleDetector(I);
    if ~isempty(bboxes)
        I = insertObjectAnnotation(I,'rectangle',bboxes,scores);
        imshow(I)
        title('Detected people and detection scores');
    end
end