import csv
import matplotlib.pyplot as plt


x = []
y = []
time = []

with open('plot.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
            #print(f'\t{row[0]} works in the {row[1]} department, and was born in {row[2]}.')
            x.append(row[0])
            y.append(row[1])
            time.append(row[2])
            line_count += 1
    print(f'Processed {line_count} lines.')

# Plot X angle
plt.plot(time,x)
plt.title("X angle over time")
plt.xlabel("Time")
plt.ylabel("Angle X")

plt.savefig("y_plot.png", dpi=300)
