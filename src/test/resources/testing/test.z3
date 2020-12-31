(declare-const output1 Real)
(declare-const output2 Real)
(declare-const input Real)
(assert (= input (+ output1 output2)))

(push)
(assert (>= input -1000))
(assert (<= input 1000))

(assert (>= output1 -1000))
(assert (<= output1 1000))

(assert (>= output2 -1000))
(assert (<= output2 1000))

(check-sat)
(get-model)
(pop)

(echo "Port Range Validation")
(push)
(echo "output2 lowerLimit range")
(assert (>= input -1000))
(assert (<= input 1000))

(assert (>= output1 -1000))
(assert (<= output1 1000))

(assert (< output2 -1000))
(check-sat)
(get-model)
(pop)

(push)
(echo "output2 upper range")
(assert (>= input -1000))
(assert (<= input 1000))

(assert (>= output1 -1000))
(assert (<= output1 1000))

(assert (> output2 1000))
(check-sat)
(get-model)
(pop)
