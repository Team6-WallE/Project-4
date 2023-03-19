# Generated by Django 4.1.6 on 2023-03-17 12:08

from django.db import migrations, models
import django.db.models.deletion


class Migration(migrations.Migration):

    initial = True

    dependencies = [
    ]

    operations = [
        migrations.CreateModel(
            name='QuestionTable',
            fields=[
                ('question_id', models.AutoField(primary_key=True, serialize=False)),
                ('questions', models.TextField()),
            ],
        ),
        migrations.CreateModel(
            name='AnswerTable',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('answers', models.TextField()),
                ('question_id', models.ForeignKey(null=True, on_delete=django.db.models.deletion.SET_NULL, to='faq.questiontable')),
            ],
        ),
    ]
