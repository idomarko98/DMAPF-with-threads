from multiprocessing import Queue, JoinableQueue, SimpleQueue


class Picklable_Priorty_Queue:

    def __init__(self):
        self.queue = Queue()

    def put(self, state):
        temp_queue = Queue()
        inserted = False
        # while loop to get the new item in the temp queue
        while not self.queue.empty():
            temp_state = self.queue.get()
            if state[0] < temp_state[0]:  # Not sure what should be here
                temp_queue.put(state)
                temp_queue.put(temp_state)
                inserted = True
                break
            else:
                temp_queue.put(temp_state)

        # if the new state is the smallest, then it wouldn't been added yet
        if not inserted:
            temp_queue.put(state)

        # while loop to move all item left from original queue to temp queue
        while not self.queue.empty():
            temp_queue.put(self.queue.get())

        # move all items back to the original queue
        while not temp_queue.empty():
            self.queue.put(temp_queue.get())

        # temp_queue.close()
        # temp_queue.task_done()
        # temp_queue.join_thread()
        # self.queue.close()
        # self.queue.join_thread()

    def get(self):
        print("get")
        return self.queue.get()

    def qsize(self):
        print("size")
        return self.queue.qsize()

    def empty(self):
        print("empty")
        return self.queue.empty()
