import matplotlib.pyplot as plt
import pandas as pd

for i in range(1, 7):
    df1 = pd.read_csv('results{}.csv'.format(i))
    mean1 = pd.DataFrame.mean(df1)['time']
    print("{1:.8f}{0}".format('\t:\tresults{}'.format(i), mean1))
    df2 = pd.read_csv('results{} distributed.csv'.format(i))
    mean2 = pd.DataFrame.mean(df2)['time']
    print("{1:.8f}{0}".format("\t:\tresults{} distributed".format(i), mean2))
    plt.ylabel('time')
    plt.plot('time', data=df1, color='blue', label='Without Threads')
    plt.plot('time', data=df2, color='green', label='With Threads')
    # plt.plot(data=df2['time'], color='green', label='With Threads')
    plt.ylabel('time (s)')
    if i > 4:
        plt.axis([0, 50, 0, i * 2])
    elif i == 4:
        plt.axis([0, 50, 0, 3])
    else:
        plt.axis([0, 50, 0, 0.4])
    plt.title('Number of agents: {}'.format(i))
    plt.legend()
    plt.grid()
    plt.savefig('results{}'.format(i), transparent=True)
    plt.show()

# for i in df1.iterrows():
#     print(i[1]['time'])


# # for i in range(1, 8):
# df1 = pd.read_csv('means.csv')
# # mean1 = pd.DataFrame.mean(df1)['time']
# # print("{1:.8f}{0}".format('\t:\tresults{}'.format(i), mean1))
# df2 = pd.read_csv('means.csv')
# plt2 = pd.DataFrame.iteritems(df1)
# print(plt2)
# # mean2 = pd.DataFrame.mean(df2)['time']
# # print("{1:.8f}{0}".format("\t:\tresults{} threads".format(i), mean2))
# plt.ylabel('time')
# plt.plot('without threads', data=df1, color='blue', label='Without Threads')
# plt.plot('with threads', data=df2, color='green', label='With Threads')
# # plt.plot(data=df2['time'], color='green', label='With Threads')
# plt.ylabel('time (s)')
# # plt.title('Number of agents: {}'.format(i))
# plt.legend()
# # plt.savefig('results{}'.format(i), transparent=True)
# plt.show()
