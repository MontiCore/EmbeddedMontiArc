// (c) https://github.com/MontiCore/monticore 

package Generation;

script ForLoop
    Q sum = 1;

    for i = 1:2:9
        sum += sum * i;
    end
end
