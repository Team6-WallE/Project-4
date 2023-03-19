from django.db import models
from datetime import datetime, timedelta

# Create your models here.
class BookTable(models.Model):
    book_id = models.AutoField(primary_key=True)
    book_name = models.CharField(max_length=200)
    author = models.CharField(max_length=20)
    isbn = models.CharField(max_length=20)

    def __str__(self):
        return str(self.book_id) + " " + str(self.book_name)
    
class MemberTable(models.Model):
    member_id = models.AutoField(primary_key=True)
    name = models.CharField(max_length=20)
    email = models.EmailField()
    
    def __str__(self):
        return str(self.member_id) + " " + str(self.name)
    
class BorrowTable(models.Model):
    book_id = models.ForeignKey(BookTable, null=True, on_delete=models.SET_NULL)
    member_id = models.ForeignKey(MemberTable, null=True, on_delete=models.SET_NULL)
    start_date = models.DateTimeField()
    return_date = models.DateTimeField()
    
    def __str__(self):
        return str(self.book_id) + " " + str(self.member_id)
    
