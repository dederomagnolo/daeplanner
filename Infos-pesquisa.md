Em termos simples, cada árvore no mapa começa como hipótese e só vira confirmada quando passa em 4 testes.

Critérios de confirmação (valores atuais do tree_stack.launch)

hits >= 3
A árvore precisa ser observada pelo menos 3 vezes.
Argumento: reduz falso positivo de detecção isolada (ruído de 1 frame).

std_xy <= 0.70
A posição estimada (x,y) não pode oscilar demais entre observações.
Argumento: tronco é objeto estático; alta variação espacial sugere associação errada, oclusão ou ruído.

std_diameter <= 0.28
O diâmetro estimado precisa ser estável ao longo do tempo.
Argumento: diâmetro de tronco não muda frame a frame; instabilidade indica mistura de objetos/segmentação ruim.

confiança média >= 0.55
A média de confiança das observações deve ser suficiente.
Argumento: combina evidência geométrica com evidência probabilística do detector.

Se falhar em qualquer um deles, continua candidate.