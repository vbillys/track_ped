from munkres import Munkres, print_matrix

matrix = [[9, 5, 1],
          [1, 3, 2],
          [.8, 7, 4]]
m = Munkres()
indexes = m.compute(matrix)
print_matrix(matrix, msg='Lowest cost through this matrix:')
total = 0
for row, column in indexes:
    value = matrix[row][column]
    total += value
    print '(%d, %d) -> %d' % (row, column, value)
print 'total cost: %d' % total
