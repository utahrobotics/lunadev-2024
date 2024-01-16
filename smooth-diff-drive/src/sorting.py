from dataclasses import dataclass, field


@dataclass(order=True)
class Row:
    loss: float
    index: int=field(compare=False)


super_log = open("super.log", "r")
lines = super_log.readlines()
skip = True
rows = []


for i, line in enumerate(lines):
    if skip:
        skip = False
        continue
    skip = True
    i = int(i / 2)
    comma_i = line.index(',')

    for j in range(comma_i - 1, -1, -1):
        if line[j] == " ":
            space_i = j
            break
    else:
        raise Exception("No Space")

    num_string = line[space_i + 1:comma_i]
    if num_string == "NaN":
        continue
    loss = float(num_string)
    rows.append(Row(loss, i + 1))


rows.sort()
for row in rows:
    print(row.index, row.loss)
